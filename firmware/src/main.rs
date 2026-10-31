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
mod ui;

static TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Build tag, carried on every [meter] console line so a paste can
/// never be mistaken for another build (the boot banner is usually lost
/// before the host opens the port).
const BUILD: &[u8] = b"r78";

use crate::ui::Press;
use crate::ui::push_pdo;
use crate::ui::Screen;
use crate::ui::Ui;
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

    // Wait for USB CDC to be configured (max 3 seconds).
    // The host needs time to enumerate and open the COM port.
    for _ in 0..30 {
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
        Timer::after_millis(50).await;
    }
    say(b"[lcd] FNB58 r78: reboot detector\r\n").await;

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

    // --- UI: POWER + PD screens; buttons navigate (r70) ---
    // Buttons: to-GND switches, internal pull-ups latched at init
    // (README pin map: LEFT=PA15, OK=PB4, BACK=PB9, RIGHT=PB8).
    let mut btns = buttons::Buttons::new();
    let mut sender = Sender::new();
    let mut ui = Ui::new();
    ui.enter(Screen::Power);

    let mut tick: u32 = 0;
    sender.push(b"[boot] loop start
");

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

        if ok && ui.screen == Screen::Power {
            ui.draw_power(mv, ua, mw);
        }
        // (The TPA626 console feed is gone — the screen shows the values;
        // the [pd] diag line below is the periodic console heartbeat and
        // carries the build tag.)

        // PD poll: parse Source_Capabilities as they appear on the CC
        // lines (the source re-sends them whenever the DUT hard-resets).
        if matches!(pd::poll(&mut tpa, &mut caps, &mut listener).await, Ok(true)) {
            report_caps(&caps, &mut sender);
        }
        // Tester-initiated discovery, automatic: while no caps have been
        // captured, ask the source every 5 s (Get_Source_Cap + Request of
        // the first fixed PDO inside discover()).
        if caps.updates == 0 && tick % 50 == 0 && tick > 0 {
            match pd::discover(&mut tpa, &mut caps, &mut listener).await {
                Ok(true) => report_caps(&caps, &mut sender),
                Ok(false) => {}
                Err(_) => sender.push(b"[pd] auto scan FAIL\r\n"),
            }
        }
        // Periodic diag heartbeat with the build tag, queue depth and
        // send-failure count (f>0 = the host is not collecting packets).
        if tick % 20 == 0 {
            let mut d = [0u8; 64];
            let mut n = 0;
            for &b in b"[pd " {
                d[n] = b;
                n += 1;
            }
            for &b in BUILD {
                d[n] = b;
                n += 1;
            }
            for &b in b"] t=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, tick % 100000);
            for &b in b" q=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, sender.qlen as u32);
            for &b in b" f=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, sender.take_fails());
            n = listener.write_diag(&mut d, n);
            d[n] = b'\r';
            d[n + 1] = b'\n';
            sender.push(&d[..n + 2]);
        }
        if ui.screen == Screen::Pd {
            ui.draw_pd(&caps);
        }

        // Buttons: debounced active-low; events to the console queue,
        // presses drive the UI screens.
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

            if !down {
                continue;
            }
            match ui.on_press(i) {
                Press::Switch(screen) => ui.enter(screen),
                Press::Discover => {
                    // Active discovery (PD switch ON, no DUT):
                    // Get_Source_Cap -> caps -> Request the first
                    // fixed PDO, so the source holds the contract.
                    caps = pd::SourceCaps::default();
                    listener = pd::Listener::default();
                    match pd::init(&mut tpa).await {
                        Ok(_) => match pd::discover(&mut tpa, &mut caps, &mut listener).await {
                            Ok(true) => {
                                let mut d = [0u8; 64];
                                let mut n = 0;
                                for &b in b"[pd] scan ok, contract requested\r\n" {
                                    d[n] = b;
                                    n += 1;
                                }
                                sender.push(&d[..n]);
                            }
                            Ok(false) => sender.push(b"[pd] scan: no reply\r\n"),
                            Err(_) => sender.push(b"[pd] scan FAIL\r\n"),
                        },
                        Err(_) => sender.push(b"[pd] reinit FAIL\r\n"),
                    }
                }
                Press::None => {}
            }
        }

        sender.drain();

        tick = tick.wrapping_add(1);
        Timer::after_millis(100).await;
    }
}

/// Report new source capabilities to the console queue (the PD
/// screen in ui.rs owns the on-screen list).
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

}

/// CDC line queue. The vendor send rejects a second call while one is in
/// flight, so every console line the main loop produces (meter status,
/// button events, PD capabilities) is queued here and drained with 500 ms
/// gaps — one drain point guarantees the sends never collide.
pub(crate) struct Sender {
    queue: [[u8; 64]; 20],
    qhead: usize,
    qlen: usize,
    /// Send attempts rejected by the busy endpoint since last take_fails().
    fails: u32,
}

impl Sender {
    fn new() -> Self {
        Self {
            queue: [[0; 64]; 20],
            qhead: 0,
            qlen: 0,
            fails: 0,
        }
    }

    pub(crate) fn push(&mut self, line: &[u8]) {
        if self.qlen == self.queue.len() || line.len() > 64 {
            return; // backlog or oversize: drop, the next report suffices
        }
        let slot = (self.qhead + self.qlen) % self.queue.len();
        self.queue[slot] = [0; 64];
        self.queue[slot][..line.len()].copy_from_slice(line);
        self.qlen += 1;
    }

    /// Try to send one queued line. Non-blocking: the vendor send rejects
    /// (nonzero) while the previous packet is still uncollected by the
    /// host, and the head line then retries on the next 100 ms tick — a
    /// slow host delays lines instead of losing them.
    fn drain(&mut self) {
        if self.qlen == 0 {
            return;
        }
        let line = self.queue[self.qhead];
        let len = line.iter().position(|&b| b == 0).unwrap_or(line.len());
        // SAFETY: single-threaded; the send is an endpoint buffer copy.
        let ok = unsafe { fnb58_usb_send(line.as_ptr(), len as u16) } == 0;
        if ok {
            self.qhead = (self.qhead + 1) % self.queue.len();
            self.qlen -= 1;
        } else {
            self.fails += 1;
        }
    }

    /// Rejected-send count since the last call.
    pub(crate) fn take_fails(&mut self) -> u32 {
        let f = self.fails;
        self.fails = 0;
        f
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
