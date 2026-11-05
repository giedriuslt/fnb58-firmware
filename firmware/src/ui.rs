//! Two-screen UI in the kws-x1 style: a POWER monitor (live V/A/W) and a
//! PD screen (the engine view from the vendored fusb302-pd session,
//! r113). LEFT/RIGHT (and BACK) switch screens; OK on the PD screen
//! signals the PD task. Rendering goes through the Rust LCD driver —
//! every string here must only contain glyphs from lcd.rs FONT_CHARS.

use crate::buttons;
use crate::lcd::Lcd;
use crate::pd_task::View;

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
    /// PD-view identity at the last redraw (values that change rarely).
    pd_phase: u8,
    pd_mv: u16,
    pd_ma: u16,
    pd_err: u16,
}

/// Draw a NUL-terminated copy of `s` (truncated to fit the buffer).
async fn text(lcd: &mut Lcd, x: u16, y: u16, s: &[u8], scale: u8, color: u16) {
    let mut buf = [0u8; 24];
    let n = s.len().min(23);
    buf[..n].copy_from_slice(&s[..n]);
    lcd.draw_text(x, y, &buf, scale, color, BLACK).await;
}

impl Ui {
    pub fn new() -> Self {
        Self {
            screen: Screen::Power,
            pd_phase: 0xFF,
            pd_mv: 0xFFFF,
            pd_ma: 0xFFFF,
            pd_err: 0xFFFF,
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
    pub async fn enter(&mut self, lcd: &mut Lcd, screen: Screen) {
        self.screen = screen;
        // Force the next screen-specific redraw.
        self.pd_phase = 0xFF;
        lcd.fill(BLACK).await;
        match screen {
            Screen::Power => {
                text(lcd, 8, 8, b"POWER", 2, WHITE).await;
                text(lcd, 272, 8, b"1/2", 2, GRAY).await;
            }
            Screen::Pd => {
                text(lcd, 8, 8, b"PD", 2, WHITE).await;
                text(lcd, 272, 8, b"2/2", 2, GRAY).await;
                text(lcd, 8, 216, b"OK: SCAN", 2, GRAY).await;
            }
        }
    }

    /// Show the reset-cause flags in the title bar so a reboot loop is
    /// visible standalone (no PC needed).
    pub async fn draw_rst(&self, lcd: &mut Lcd, flags: u8) {
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
        lcd.draw_text(180, 8, &line, 2, GRAY, BLACK).await;
    }

    /// Power screen: the three big live values (12-char padded overdraws,
    /// so shrinking values leave no ghosts).
    pub async fn draw_power(&self, lcd: &mut Lcd, mv: u32, ua: i32, mw: i32) {
        // Voltage: "20.000 V"
        let mut l1 = Line12::new();
        l1.num_u32(mv, 3);
        l1.push(b" V");
        l1.pad();
        lcd.draw_text(20, 50, l1.bytes(), 3, GREEN, BLACK).await;

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
        lcd.draw_text(20, 110, l2.bytes(), 3, YELLOW, BLACK).await;

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
        lcd.draw_text(20, 170, l3.bytes(), 3, CYAN, BLACK).await;
    }

    /// PD screen: the engine view (vendored fusb302-pd session). Redrawn
    /// only when a tracked value changes.
    pub async fn draw_pd(&mut self, lcd: &mut Lcd, view: &View) {
        let phase = view.phase.load(core::sync::atomic::Ordering::Relaxed);
        let mv = view.selected_mv.load(core::sync::atomic::Ordering::Relaxed);
        let ma = view.selected_ma.load(core::sync::atomic::Ordering::Relaxed);
        let err = view.error.load(core::sync::atomic::Ordering::Relaxed);
        if phase == self.pd_phase && mv == self.pd_mv && ma == self.pd_ma && err == self.pd_err {
            return;
        }
        self.pd_phase = phase;
        self.pd_mv = mv;
        self.pd_ma = ma;
        self.pd_err = err;
        lcd.fill_window(0, 32, 319, 210, BLACK).await;

        const NAMES: [&[u8]; 13] = [
            b"WAIT", b"PASSIVE", b"FAIL", b"SKIP", b"MEAS", b"GUARD", b"VERIFY", b"RX",
            b"CAPTURED", b"NOCAPS", b"NEG", b"CONTRACT", b"HOLD",
        ];
        let name = NAMES.get(phase as usize).copied().unwrap_or(b"?");
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"PD " {
            line[n] = b;
            n += 1;
        }
        line[n..n + name.len()].copy_from_slice(name);
        n += name.len();
        line[n] = 0;
        text(lcd, 8, 40, &line, 3, WHITE).await;

        // Selected voltage/current: "SEL 5.0V 3.0A".
        let mut l = Line12::new();
        for &b in b"SEL " {
            l.push(&[b]);
        }
        if mv >= 1000 {
            // r142: mv is MILLIVOLTS - scale to volts first (5000 mV
            // printed as "500.0V" before). Line12 caps at 12 chars, so
            // whole volts + one-decimal amps fit: "SEL 5V 3.0A".
            l.num_u32(u32::from(mv) / 1000, 0);
            l.push(b"V ");
        } else {
            l.push(b"- ");
        }
        if ma >= 1000 {
            l.num_u32(u32::from(ma) / 100, 1);
            l.push(b"A");
        } else {
            l.push(b"-A");
        }
        l.pad();
        lcd.draw_text(8, 90, l.bytes(), 2, GREEN, BLACK).await;

        // Flags: "ACC:1 RDY:0 CON:1".
        let acc = u8::from(view.accept.load(core::sync::atomic::Ordering::Relaxed));
        let rdy = u8::from(view.psrdy.load(core::sync::atomic::Ordering::Relaxed));
        let con = u8::from(view.contract.load(core::sync::atomic::Ordering::Relaxed));
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"ACC:1 RDY:1 CON:1" {
            line[n] = b;
            n += 1;
        }
        line[4] = b'0' + acc;
        line[10] = b'0' + rdy;
        line[16] = b'0' + con;
        line[n] = 0;
        text(lcd, 8, 130, &line, 2, CYAN).await;

        // Diagnostics: "REV:3 ERR:0 PK:12".
        let rev = view.peer_rev.load(core::sync::atomic::Ordering::Relaxed);
        let pk = view.rx_packets.load(core::sync::atomic::Ordering::Relaxed);
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"REV:0 ERR:0 PK:0" {
            line[n] = b;
            n += 1;
        }
        line[4] = b'0' + (rev % 10);
        line[11] = b'0' + (err % 10) as u8;
        let mut pk_n = 0;
        let mut v = pk;
        while v > 0 && n + 1 < line.len() {
            line[n - pk_n] = b'0' + (v % 10) as u8;
            pk_n += 1;
            v /= 10;
        }
        if pk == 0 {
            line[n - 1] = b'0';
        }
        line[n] = 0;
        text(lcd, 8, 160, &line, 2, YELLOW).await;
    }
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

    pub fn bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}
