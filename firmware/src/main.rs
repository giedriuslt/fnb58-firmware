//! FNB58 r62: the working meter screen — TPA626 V/A/W (Rust embassy I2C
//! driver, ACK-state fix verified), FUSB302 passive PD listener with the
//! source-capability strip on the LCD, and the four verified buttons
//! (LEFT=PA15, OK=PB4, BACK=PB9, RIGHT=PB8) shown on-screen.

#![no_std]
#![no_main]

mod board;
mod buttons;
mod pd;
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
    say(b"[lcd] FNB58 r69: pdo amp display fix\r\n").await;

    unsafe extern "C" {
        fn fnb58_i2c_init();
        fn fnb58_i2c_probe(addr: u16) -> u32;
        fn fnb58_i2c_read(addr: u16, reg: u8, out: *mut u8, len: u16) -> u32;
    }

    // Hardware I2C health check: probe + a REAL hw register read of the
    // TPA626 manufacturer id (expect 55 49).
    {
        unsafe { fnb58_i2c_init() };
        let st = unsafe { fnb58_i2c_probe(0x40) };
        let mut mfr = [0u8; 2];
        let e = unsafe { fnb58_i2c_read(0x40, 0xFE, mfr.as_mut_ptr(), 2) };
        let mut d = [0u8; 96];
        let mut n = 0;
        for &b in b"[i2c] hw FIXED st=" {
            d[n] = b;
            n += 1;
        }
        n = push_dec(&mut d, n, st);
        for &b in b" rdE=" {
            d[n] = b;
            n += 1;
        }
        n = push_dec(&mut d, n, e);
        for &b in b" mfr=" {
            d[n] = b;
            n += 1;
        }
        n = push_hex8(&mut d, n, mfr[0] as u32);
        n = push_hex8(&mut d, n, mfr[1] as u32);
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

    // --- r55: meter transport on the Rust embassy I2C driver ---
    // First hardware run of the HAL driver (host-tested only until now).
    // The C glue's boot check above already proved the bus; this is the
    // Rust engine doing the live measurement.
    use embedded_hal_async::i2c::Operation;
    use embedded_hal_async::i2c::I2c as _;
    use embassy_at32::gpio::{AlternatePin, Drive, Port};
    use embassy_at32::i2c::{I2c, Speed};

    // PB6/PB7 as I2C1 mux open-drain, then the async master at 100 kHz.
    let _scl = unsafe { AlternatePin::new_open_drain(Port::B, 6, Drive::Moderate) };
    let _sda = unsafe { AlternatePin::new_open_drain(Port::B, 7, Drive::Moderate) };
    let mut tpa = match unsafe { I2c::new_i2c1(clocks.pclk1_hz, Speed::Standard) } {
        Ok(i2c) => i2c,
        Err(_) => {
            say(b"[meter] rust i2c config FAIL\r\n").await;
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };

    async fn tpa_write(tpa: &mut I2c, reg: u8, val: u16) -> Result<(), embassy_at32::i2c::Error> {
        let mut wr = [reg, (val >> 8) as u8, val as u8];
        tpa.transaction(0x40, &mut [Operation::Write(&mut wr)]).await
    }
    async fn tpa_read(tpa: &mut I2c, reg: u8) -> Result<u16, embassy_at32::i2c::Error> {
        let mut reg_buf = [reg];
        let mut buf = [0u8; 2];
        tpa.transaction(
            0x40,
            &mut [Operation::Write(&mut reg_buf), Operation::Read(&mut buf)],
        )
        .await?;
        Ok(((buf[0] as u16) << 8) | buf[1] as u16)
    }

    // Identify + configure the TPA626 (INA226 protocol).
    match tpa_read(&mut tpa, 0xFE).await {
        Ok(0x5549) => say(b"[meter] rust i2c OK mfr=5549\r\n").await,
        Ok(v) => {
            let mut d = [0u8; 96];
            let mut n = 0;
            for &b in b"[meter] rust i2c mfr ODD=" {
                d[n] = b;
                n += 1;
            }
            n = push_hex8(&mut d, n, v as u32);
            d[n] = 13;
            d[n + 1] = 10;
            unsafe { fnb58_usb_send(d.as_ptr(), (n + 2) as u16) };
            Timer::after_millis(500).await;
        }
        Err(_) => say(b"[meter] rust i2c read FAIL\r\n").await,
    }
    let _ = tpa_write(&mut tpa, 0x00, 0x4897).await; // AVG=64, cont
    let cal: u16 = (5_120_000u32 / 500u32) as u16; // 10 mOhm shunt, 50 uA LSB
    let _ = tpa_write(&mut tpa, 0x05, cal).await;
    say(b"[meter] rust tpa configured\r\n").await;

    // --- FUSB302 passive PD listener ---
    let mut caps = pd::SourceCaps::default();
    let mut listener = pd::Listener::default();
    match pd::init(&mut tpa).await {
        Ok(id) => {
            let mut d = [0u8; 96];
            let mut n = 0;
            for &b in b"[pd] fusb302 id=" {
                d[n] = b;
                n += 1;
            }
            n = push_hex8(&mut d, n, id as u32);
            d[n] = 13;
            d[n + 1] = 10;
            unsafe { fnb58_usb_send(d.as_ptr(), (n + 2) as u16) };
            Timer::after_millis(500).await;
        }
        Err(_) => say(b"[pd] fusb302 init FAIL\r\n").await,
    }

    // Meter loop (same as r46).
    unsafe { fnb58_lcd_fill(0x0000) };

    // --- r62: the four verified buttons + a single CDC send queue ---
    // Buttons: to-GND switches, internal pull-ups latched at init
    // (README pin map: LEFT=PA15, OK=PB4, BACK=PB9, RIGHT=PB8).
    let mut btns = buttons::Buttons::new();
    let mut sender = Sender::new();

    // Fixed-width opaque text lines (12 chars incl. trailing spaces) so
    // updates fully overwrite the previous value.
    const BLACK: u16 = 0x0000;
    let mut tick: u32 = 0;
    loop {
        let mut mv: u32 = 0;
        let mut ua: i32 = 0;
        let mut mw: i32 = 0;
        let ok = match (
            tpa_read(&mut tpa, 0x02).await,
            tpa_read(&mut tpa, 0x04).await,
            tpa_read(&mut tpa, 0x03).await,
        ) {
            (Ok(bus), Ok(cur), Ok(pwr)) => {
                mv = (bus as u32 * 5) / 4; // 1.25 mV/LSB
                ua = (cur as i16 as i32) * 50; // 50 uA/LSB
                mw = ((pwr as i16 as i32) * 5) / 4; // 1.25 mW/LSB
                true
            }
            _ => false,
        };

        if ok {
            // Voltage: "20.000 V"
            let mut l1 = Line12::new();
            l1.num_u32(mv, 3);
            l1.push(b" V");
            l1.pad();
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
            l2.pad();
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
            l3.pad();
            unsafe {
                fnb58_lcd_draw_text(20, 170, l3.as_ptr(), 3, 0x07FF, BLACK);
            }

            // Console status ~every 2 s (CDC needs the 500 ms gap).
            if tick % 20 == 0 {
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
                sender.push(&dbg[..n + 2]);
            }
        } else if tick % 20 == 0 {
            sender.push(b"[meter] tpa626 read FAIL\r\n");
        }

        // PD poll: parse Source_Capabilities as they appear on the CC
        // lines (the source re-sends them whenever the DUT hard-resets).
        if matches!(pd::poll(&mut tpa, &mut caps, &mut listener).await, Ok(true)) {
            report_caps(&caps, &mut sender);
        }
        if tick % 20 == 0 {
            let mut d = [0u8; 64];
            let n = listener.write_diag(&mut d);
            sender.push(&d[..n]);
        }

        // Buttons: debounced active-low; LCD indicator top-left, events
        // to the console queue.
        while let Some((i, down)) = btns.poll() {
            let name = buttons::Buttons::name(i);
            let mut line = [0u8; 64];
            let mut w = 0;
            for &b in b"[btn] " {
                line[w] = b;
                w += 1;
            }
            if down {
                line[w..w + name.len()].copy_from_slice(name);
                w += name.len();
            }
            for &b in if down { &b" down\r\n"[..] } else { &b" up\r\n"[..] } {
                line[w] = b;
                w += 1;
            }
            sender.push(&line[..w]);

            // LCD indicator: "BTN: NAME" while held, fully blanked on
            // release (12-char overdraw erases the previous label).
            let mut lcd = [0u8; 13];
            let mut n = 0;
            if down {
                for &b in b"BTN: " {
                    lcd[n] = b;
                    n += 1;
                }
                for &b in name {
                    if n < 12 {
                        lcd[n] = b;
                        n += 1;
                    }
                }
            }
            while n < 12 {
                lcd[n] = b' ';
                n += 1;
            }
            lcd[12] = 0;
            let color: u16 = if down { 0xFFE0 } else { BLACK };
            unsafe {
                fnb58_lcd_draw_text(20, 0, lcd.as_ptr(), 2, color, BLACK);
            }
        }

        sender.drain().await;

        tick = tick.wrapping_add(1);
        Timer::after_millis(100).await;
    }
}

/// Append `v` with `dec` digits after the decimal point, into `buf` at
/// `pos` (generic-buffer version of Line12::num_u32).
fn push_num_dec(buf: &mut [u8], pos: usize, v: u32, dec: usize) -> usize {
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
/// `pos`; returns the new position.
fn push_pdo(buf: &mut [u8], pos: usize, pdo: &pd::Pdo) -> usize {
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

/// Report new source capabilities: full list to the console queue,
/// summary lines to the LCD's bottom strip.
fn report_caps(caps: &pd::SourceCaps, out: &mut Sender) {
    // Console header.
    let mut d = [0u8; 96];
    let mut n = 0;
    for &b in b"[pd] src caps " {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, caps.n as u32);
    for &b in b" PDOs" {
        d[n] = b;
        n += 1;
    }
    d[n] = 13;
    d[n + 1] = 10;
    n += 2;
    out.push(&d[..n]);

    for i in 0..caps.n {
        let mut d = [0u8; 96];
        let mut n = 0;
        for &b in b"[pd]  " {
            d[n] = b;
            n += 1;
        }
        n = push_pdo(&mut d, n, &caps.pdos[i]);
        d[n] = 13;
        d[n + 1] = 10;
        n += 2;
        out.push(&d[..n]);
    }

    // LCD summary: highest fixed profile + first PPS, bottom strip.
    unsafe { fnb58_lcd_fill_window(0, 196, 319, 239, 0x0000) };
    let mut best: Option<&pd::Pdo> = None;
    let mut pps: Option<&pd::Pdo> = None;
    for i in 0..caps.n {
        let pdo = &caps.pdos[i];
        if pdo.is_pps {
            if pps.is_none() {
                pps = Some(pdo);
            }
        } else if best.map(|b| pdo.mv > b.mv).unwrap_or(true) {
            best = Some(pdo);
        }
    }
    if let Some(fixed) = best {
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"PD " {
            line[n] = b;
            n += 1;
        }
        n = push_pdo(&mut line, n, fixed);
        unsafe {
            fnb58_lcd_draw_text(8, 200, line.as_ptr(), 2, 0xFFFF, 0x0000);
        }
    }
    if let Some(pps) = pps {
        let mut line = [0u8; 24];
        let mut n = 0;
        n = push_pdo(&mut line, n, pps);
        unsafe {
            fnb58_lcd_draw_text(8, 220, line.as_ptr(), 2, 0xF81F, 0x0000);
        }
    }
}

/// CDC line queue. The vendor send rejects a second call while one is in
/// flight, so every console line the main loop produces (meter status,
/// button events, PD capabilities) is queued here and drained with 500 ms
/// gaps — one drain point guarantees the sends never collide.
struct Sender {
    queue: [[u8; 64]; 20],
    qhead: usize,
    qlen: usize,
}

impl Sender {
    fn new() -> Self {
        Self {
            queue: [[0; 64]; 20],
            qhead: 0,
            qlen: 0,
        }
    }

    fn push(&mut self, line: &[u8]) {
        if self.qlen == self.queue.len() || line.len() > 64 {
            return; // backlog or oversize: drop, the next report suffices
        }
        let slot = (self.qhead + self.qlen) % self.queue.len();
        self.queue[slot] = [0; 64];
        self.queue[slot][..line.len()].copy_from_slice(line);
        self.qlen += 1;
    }

    async fn drain(&mut self) {
        while self.qlen > 0 {
            let line = self.queue[self.qhead];
            let len = line.iter().position(|&b| b == 0).unwrap_or(line.len());
            // SAFETY: single-threaded; one send per 500 ms gap.
            unsafe { fnb58_usb_send(line.as_ptr(), len as u16) };
            Timer::after_millis(500).await;
            self.qhead = (self.qhead + 1) % self.queue.len();
            self.qlen -= 1;
        }
    }
}

/// Append decimal representation of `v` to `buf` at `pos`, returning the
/// new position (uncapped by design; callers size buffers generously).
fn push_dec(buf: &mut [u8], pos: usize, v: u32) -> usize {
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
fn push_hex8(buf: &mut [u8], pos: usize, v: u32) -> usize {
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

    /// Space-pad to the full 12 chars so a redraw erases the previous
    /// (possibly longer) value.
    fn pad(&mut self) {
        while self.len < 12 {
            self.buf[self.len] = b' ';
            self.len += 1;
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
