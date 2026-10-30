//! FNB58 LCD bring-up: ST7789V 320x240 on SPI2 (verified pin map in
//! c/fnb58_lcd.c), color-bar test pattern drawn at boot. CDC echo after.

#![no_std]
#![no_main]

mod board;
mod buttons;
mod pin_trace;

static TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

use embassy_executor::Spawner;
use embassy_time::Timer;

// --- Vendor C stack glue (compiled by build.rs into libvendorusb.a) ---

unsafe extern "C" {
    fn fnb58_usb_init();
    fn fnb58_usb_recv(buffer: *mut u8) -> u16;
    fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
    fn fnb58_usb_configured() -> u32;
    fn fnb58_lcd_init();
    fn fnb58_lcd_test_pattern();
}

/// Called from the vendor IRQ handlers.
#[unsafe(no_mangle)]
extern "C" fn fnb58_usbd_event_hook() {}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Crystal clock: 192 MHz PLL, USB /4.
    let Ok((clocks, _)) =
        (unsafe { embassy_at32::clocks::configure_usb(board::HEXT_HZ) })
    else {
        loop {
            cortex_m::asm::nop();
        }
    };
    unsafe { embassy_at32::init(clocks) };
    unsafe { cortex_m::interrupt::enable() };

    // Unmask USB interrupt lines.
    use embassy_at32::pac::at32f403a::interrupt;
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_L_CAN1_RX0);
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_H_CAN1_TX);
    }

    // Vendor USB init (CDC class).
    unsafe {
        fnb58_usb_init();
    }

    // Button monitor init.
    // Button monitor init.
    buttons::init();

    // Wait for USB CDC to be configured (max 15 seconds).
    // The host needs time to enumerate and open the COM port.
    for _ in 0..150 {
        unsafe {
            if fnb58_usb_configured() == 1 {
                break;
            }
        }
        Timer::after_millis(100).await;
    }

    // LCD bring-up: ST7789V init + color-bar test pattern.
    async fn say(msg: &'static [u8]) {
        unsafe { fnb58_usb_send(msg.as_ptr(), msg.len() as u16) };
        Timer::after_millis(500).await;
    }
    unsafe extern "C" {
        fn fnb58_measure_init();
        fn fnb58_measure_read(
            mv: *mut u32,
            ua: *mut i32,
            mw: *mut i32,
        ) -> u32;
        fn fnb58_lcd_fill(color: u16);
        fn fnb58_lcd_draw_text(
            x: u16,
            y: u16,
            s: *const u8,
            scale: u8,
            fg: u16,
            bg: u16,
        ) -> u16;
    }

    say(b"[lcd] FNB58 r53: true 192MHz + meter\r\n").await;

    unsafe extern "C" {
        fn fnb58_i2c_init();
        fn fnb58_i2c_probe(addr: u16) -> u32;
        fn fnb58_i2c_bitbang_read16(addr: u16, reg: u8, val: *mut u16) -> u32;
    }

    // One hw-I2C health check at the new true 192 MHz clock.
    {
        unsafe { fnb58_i2c_init() };
        let st = unsafe { fnb58_i2c_probe(0x40) };
        let mut mfr: u16 = 0;
        let e = unsafe { fnb58_i2c_bitbang_read16(0x40, 0xFE, &mut mfr) };
        let mut d = [0u8; 96];
        let mut n = 0;
        for &b in b"[clk] 192M fix, hw i2c st=" {
            d[n] = b;
            n += 1;
        }
        n = push_dec(&mut d, n, st);
        for &b in b" bbE=" {
            d[n] = b;
            n += 1;
        }
        n = push_dec(&mut d, n, e);
        for &b in b" mfr=" {
            d[n] = b;
            n += 1;
        }
        n = push_hex8(&mut d, n, (mfr >> 8) as u32);
        n = push_hex8(&mut d, n, (mfr & 0xFF) as u32);
        d[n] = 13;
        d[n + 1] = 10;
        n += 2;
        unsafe { fnb58_usb_send(d.as_ptr(), n as u16) };
        Timer::after_millis(500).await;
    }

    unsafe { fnb58_lcd_init() };
    say(b"[lcd] init done, drawing bars\r\n").await;
    unsafe { fnb58_lcd_test_pattern() };
    say(b"[lcd] pattern drawn\r\n").await;
    Timer::after_millis(1500).await;

    unsafe { fnb58_lcd_fill(0x0000) };
    unsafe { fnb58_measure_init() };
    say(b"[meter] bitbang tpa init\r\n").await;

    // Meter loop (same as r46).
    unsafe { fnb58_lcd_fill(0x0000) };

    // Fixed-width opaque text lines (12 chars incl. trailing spaces) so
    // updates fully overwrite the previous value.
    const BLACK: u16 = 0x0000;
    let mut tick: u32 = 0;
    loop {
        let mut mv: u32 = 0;
        let mut ua: i32 = 0;
        let mut mw: i32 = 0;
        let ok = unsafe { fnb58_measure_read(&mut mv, &mut ua, &mut mw) } == 0;

        if ok {
            // Voltage: "20.000 V"
            let mut l1 = Line12::new();
            l1.num_u32(mv, 3);
            l1.push(b" V");
            unsafe {
                fnb58_lcd_draw_text(20, 30, l1.as_ptr(), 3, 0x07E0, BLACK);
            }

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
            unsafe {
                fnb58_lcd_draw_text(20, 100, l2.as_ptr(), 3, 0xFFE0, BLACK);
            }

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
            unsafe {
                fnb58_lcd_draw_text(20, 170, l3.as_ptr(), 3, 0x07FF, BLACK);
            }

            // Console status ~every 2 s (CDC needs the 500 ms gap).
            if tick % 8 == 0 {
                let mut dbg = [0u8; 64];
                let mut n = 0;
                for &b in b"[meter] " {
                    dbg[n] = b;
                    n += 1;
                }
                for &b in l1.bytes() {
                    dbg[n] = b;
                    n += 1;
                }
                dbg[n] = b' ';
                n += 1;
                dbg[n] = b'|';
                n += 1;
                dbg[n] = b' ';
                n += 1;
                for &b in l2.bytes() {
                    dbg[n] = b;
                    n += 1;
                }
                dbg[n] = b'\r';
                dbg[n + 1] = b'\n';
                unsafe { fnb58_usb_send(dbg.as_ptr(), (n + 2) as u16) };
            }
        } else if tick % 8 == 0 {
            say(b"[meter] tpa626 read FAIL\r\n").await;
        }

        tick = tick.wrapping_add(1);
        Timer::after_millis(250).await;
    }
}

/// Append decimal representation of `v` to `buf` at `pos`, returning the
/// new position (uncapped by design; callers size buffers generously).
fn push_dec(buf: &mut [u8; 96], pos: usize, v: u32) -> usize {
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
    for k in (0..n).rev() {
        buf[p] = tmp[k];
        p += 1;
    }
    p
}

/// Append `v` as two lowercase hex digits.
fn push_hex8(buf: &mut [u8; 96], pos: usize, v: u32) -> usize {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    buf[pos] = HEX[((v >> 4) & 0xF) as usize];
    buf[pos + 1] = HEX[(v & 0xF) as usize];
    pos + 2
}

/// 12-char NUL-terminated line buffer for the meter rows.
struct Line12 {
    buf: [u8; 13],
    len: usize,
}

impl Line12 {
    const fn new() -> Self {
        Self {
            buf: [0u8; 13],
            len: 0,
        }
    }

    fn push(&mut self, s: &[u8]) {
        for &b in s {
            if self.len < 12 {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
    }

    fn sign(&mut self, neg: bool) {
        if neg {
            self.push(b"-");
        }
    }

    /// Append `v` with `dec` digits after the decimal point.
    fn num_u32(&mut self, v: u32, dec: usize) {
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

    fn as_ptr(&self) -> *const u8 {
        self.buf.as_ptr()
    }

    fn bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

#[cortex_m_rt::exception]
fn SysTick() {
    embassy_at32::time_driver::systick_tick();
}

/// Drop the D+ pull-up on fault. The device VANISHES from the host.
fn fault_halt() -> ! {
    unsafe {
        let ctrl = (0x4000_5c00usize + 0x40) as *mut u32;
        core::ptr::write_volatile(ctrl, core::ptr::read_volatile(ctrl) | 0b10);
        let cfg = (0x4000_5c00usize + 0x60) as *mut u32;
        core::ptr::write_volatile(cfg, core::ptr::read_volatile(cfg) | 0b10);
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    fault_halt()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    fault_halt()
}
