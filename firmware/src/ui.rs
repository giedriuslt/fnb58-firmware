//! Two-screen UI in the kws-x1 style: a POWER monitor (live V/A/W) and a
//! PD screen (source capabilities from the FUSB302 listener). LEFT/RIGHT
//! (and BACK) switch screens; OK on the PD screen requests a scan (r88:
//! the scan also powers the FUSB302 up — no boot-time PD init). Rendering
//! goes through the C 5x7 text API — every string here must only contain
//! glyphs from c/fnb58_lcd.c FONT_CHARS.

use crate::buttons;
use crate::pd::Pdo;
use crate::pd::SourceCaps;

unsafe extern "C" {
    fn fnb58_lcd_fill(color: u16);
    fn fnb58_lcd_fill_window(x0: u16, y0: u16, x1: u16, y1: u16, color: u16);
    fn fnb58_lcd_draw_text(
        x: u16,
        y: u16,
        s: *const u8,
        scale: u8,
        fg: u16,
        bg: u16,
    ) -> u16;
}

const BLACK: u16 = 0x0000;
const WHITE: u16 = 0xFFFF;
const GRAY: u16 = 0x7BEF;
const GREEN: u16 = 0x07E0;
const YELLOW: u16 = 0xFFE0;
const CYAN: u16 = 0x07FF;
const MAGENTA: u16 = 0xF81F;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Screen {
    Power,
    Pd,
}

/// What a button press means to the UI (buttons.rs order:
/// LEFT, OK, BACK, RIGHT).
pub enum Press {
    None,
    Switch(Screen),
    Discover,
}

pub struct Ui {
    pub screen: Screen,
    /// Caps identity at the last PD-list redraw (updates counter + row
    /// count, so a cleared-caps redraw also triggers).
    last_updates: u32,
    last_n: usize,
}

/// Draw a NUL-terminated copy of `s` (truncated to fit the buffer).
fn text(x: u16, y: u16, s: &[u8], scale: u8, color: u16) {
    let mut buf = [0u8; 24];
    let n = s.len().min(23);
    buf[..n].copy_from_slice(&s[..n]);
    unsafe { fnb58_lcd_draw_text(x, y, buf.as_ptr(), scale, color, BLACK) };
}

impl Ui {
    pub fn new() -> Self {
        Self {
            screen: Screen::Power,
            last_updates: 0,
            last_n: 0,
        }
    }

    /// Map a button press. LEFT = previous screen, RIGHT = next, BACK =
    /// power screen; OK means scan on the PD screen.
    pub fn on_press(&mut self, i: usize) -> Press {
        match i {
            buttons::LEFT => Press::Switch(Screen::Power),
            buttons::RIGHT => Press::Switch(Screen::Pd),
            buttons::BACK => Press::Switch(Screen::Power),
            buttons::OK if self.screen == Screen::Pd => Press::Discover,
            _ => Press::None,
        }
    }

    /// Switch screens: full clear, title and static chrome.
    pub fn enter(&mut self, screen: Screen) {
        self.screen = screen;
        // Force the next screen-specific redraw.
        self.last_updates = u32::MAX;
        unsafe { fnb58_lcd_fill(BLACK) };
        match screen {
            Screen::Power => {
                text(8, 8, b"POWER", 2, WHITE);
                text(272, 8, b"1/2", 2, GRAY);
            }
            Screen::Pd => {
                text(8, 8, b"PD", 2, WHITE);
                text(272, 8, b"2/2", 2, GRAY);
                text(8, 216, b"OK: SCAN", 2, GRAY);
            }
        }
    }

    /// Show the reset-cause flags in the title bar so a reboot loop is
    /// visible standalone (no PC needed).
    pub fn draw_rst(&self, flags: u8) {
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"rst=" {
            line[n] = b;
            n += 1;
        }
        const HEX: &[u8; 16] = b"0123456789abcdef";
        line[n] = HEX[(flags >> 4) as usize];
        line[n + 1] = HEX[(flags & 0xF) as usize];
        n += 2;
        line[n] = 0;
        unsafe { fnb58_lcd_draw_text(180, 8, line.as_ptr(), 2, GRAY, BLACK) };
    }

    /// Power screen: the three big live values (12-char padded overdraws,
    /// so shrinking values leave no ghosts).
    pub fn draw_power(&self, mv: u32, ua: i32, mw: i32) {
        // Voltage: "20.000 V"
        let mut l1 = Line12::new();
        l1.num_u32(mv, 3);
        l1.push(b" V");
        l1.pad();
        unsafe { fnb58_lcd_draw_text(20, 50, l1.as_ptr(), 3, GREEN, BLACK) };

        // Current: A (>= 1 A) else mA.
        let mut l2 = Line12::new();
        let neg = ua < 0;
        let a = (if neg { -ua } else { ua }) as u32;
        if a >= 1_000_000 {
            l2.sign(neg);
            l2.num_u32(a / 1_000, 3);
            l2.push(b" A");
        } else {
            l2.sign(neg);
            l2.num_u32(a / 100, 1);
            l2.push(b" mA");
        }
        l2.pad();
        unsafe { fnb58_lcd_draw_text(20, 110, l2.as_ptr(), 3, YELLOW, BLACK) };

        // Power: W (>= 1 W) else mW.
        let mut l3 = Line12::new();
        let negp = mw < 0;
        let p = (if negp { -mw } else { mw }) as u32;
        if p >= 1000 {
            l3.sign(negp);
            l3.num_u32(p, 3);
            l3.push(b" W");
        } else {
            l3.sign(negp);
            l3.num_u32(p, 0);
            l3.push(b" mW");
        }
        l3.pad();
        unsafe { fnb58_lcd_draw_text(20, 170, l3.as_ptr(), 3, CYAN, BLACK) };
    }

    /// PD screen: the caps list (redrawn only when the caps change) or
    /// the scan hint while nothing has been captured. OK sends
    /// Get_Source_Cap and Requests the first fixed PDO (r88).
    pub fn draw_pd(&mut self, caps: &SourceCaps) {
        if caps.updates == self.last_updates && caps.n == self.last_n {
            return;
        }
        self.last_updates = caps.updates;
        self.last_n = caps.n;
        unsafe { fnb58_lcd_fill_window(0, 32, 319, 210, BLACK) };
        if caps.n == 0 {
            text(8, 60, b"NO CAPS - PRESS OK", 2, GRAY);
            return;
        }
        for i in 0..caps.n {
            let pdo = &caps.pdos[i];
            let mut line = [0u8; 24];
            let mut n = 0;
            n = push_pdo(&mut line, n, pdo);
            line[n] = 0;
            let color = if pdo.is_pps { MAGENTA } else { WHITE };
            text(8, 40 + (i as u16) * 22, &line, 2, color);
        }
    }
}

/// Append `v` with `dec` digits after the decimal point, into `buf` at
/// `pos`. `v` is in 10^-dec units (e.g. mv/100 with dec=1 for volts).
/// Returns the new position.
pub fn push_num_dec(buf: &mut [u8], pos: usize, v: u32, dec: usize) -> usize {
    let mut tmp = [0u8; 10];
    let mut n = 0;
    let mut x = v;
    if x == 0 {
        tmp[0] = b'0';
        n = 1;
    } else {
        while x > 0 && n < 10 {
            tmp[n] = b'0' + (x % 10) as u8;
            x /= 10;
            n += 1;
        }
    }
    let mut p = pos;
    if n <= dec {
        buf[p] = b'0';
        p += 1;
        if dec > 0 {
            buf[p] = b'.';
            p += 1;
            for _ in 0..(dec - n) {
                buf[p] = b'0';
                p += 1;
            }
        }
    } else {
        let mut k = n;
        while k > dec {
            k -= 1;
            buf[p] = tmp[k];
            p += 1;
        }
        if dec > 0 {
            buf[p] = b'.';
            p += 1;
        }
    }
    if dec > 0 {
        let mut k = dec.min(n);
        while k > 0 {
            k -= 1;
            buf[p] = tmp[k];
            p += 1;
        }
    }
    p
}

/// Format one PDO as "x.xV y.yA" / "PPS a.a-b.bV y.yA" into `buf` at
/// `pos`; returns the new position (NUL terminator NOT appended).
pub fn push_pdo(buf: &mut [u8], pos: usize, pdo: &Pdo) -> usize {
    let mut p = pos;
    if pdo.is_pps {
        for &b in b"PPS " {
            buf[p] = b;
            p += 1;
        }
        p = push_num_dec(buf, p, pdo.min_mv / 100, 1);
        buf[p] = b'-';
        p += 1;
    }
    p = push_num_dec(buf, p, pdo.mv / 100, 1);
    buf[p] = b'V';
    p += 1;
    buf[p] = b' ';
    p += 1;
    if pdo.ma >= 1000 {
        p = push_num_dec(buf, p, pdo.ma / 100, 1);
        buf[p] = b'A';
    } else {
        p = push_num_dec(buf, p, pdo.ma, 0);
        buf[p] = b'm';
        buf[p + 1] = b'A';
        p += 1;
    }
    p + 1
}

/// 12-char NUL-terminated line buffer for the meter rows.
pub struct Line12 {
    buf: [u8; 13],
    len: usize,
}

impl Line12 {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; 13],
            len: 0,
        }
    }

    pub fn push(&mut self, s: &[u8]) {
        for &b in s {
            if self.len < 12 {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
    }

    pub fn sign(&mut self, neg: bool) {
        if neg {
            self.push(b"-");
        }
    }

    /// Append `v` with `dec` digits after the decimal point.
    pub fn num_u32(&mut self, v: u32, dec: usize) {
        let mut digits = [0u8; 10];
        let mut n = 0;
        let mut x = v;
        if x == 0 {
            digits[0] = b'0';
            n = 1;
        } else {
            while x > 0 && n < 10 {
                digits[n] = b'0' + (x % 10) as u8;
                x /= 10;
                n += 1;
            }
        }
        // digits[0] is the least-significant digit.
        if n <= dec {
            self.push(b"0");
            if dec > 0 {
                self.push(b".");
                for _ in 0..(dec - n) {
                    self.push(b"0");
                }
                for k in (0..n).rev() {
                    self.push(&[digits[k]]);
                }
            }
        } else {
            let mut k = n;
            while k > dec {
                k -= 1;
                self.push(&[digits[k]]);
            }
            if dec > 0 {
                self.push(b".");
                while k > 0 {
                    k -= 1;
                    self.push(&[digits[k]]);
                }
            }
        }
    }

    /// Space-pad to the full 12 chars so a redraw erases the previous
    /// (possibly longer) value.
    pub fn pad(&mut self) {
        while self.len < 12 {
            self.buf[self.len] = b' ';
            self.len += 1;
        }
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.buf.as_ptr()
    }

    pub fn bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}
