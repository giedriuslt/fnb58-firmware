//! FNB58 r102: EXINT everywhere. The user traced the FUSB302 INT_N to
//! PC13 (LQFP48 pin 2 - the last untraced pin of the PD path). Buttons
//! (PA15/PB4/PB9/PB8) and PC13 now run on external interrupts: each line
//! latches in the ISR and wakes the main loop through a signal - buttons
//! double as the interrupt canary (one '[exti] isr line=N ok' line per
//! line on its first ISR), the PD poll runs immediately on every INT
//! edge instead of waiting the 10 ms tick, and an i=N counter in the
//! heartbeat shows whether the FUSB302 ever asserts INT_N. PD RX (r100:
//! spontaneous caps parsed correctly) was poll-only; the INT path is
//! what a real sink needs to hit tSenderResponse reliably.
//! (History r88-r101: PD-on-request bisect chain - boot-time init
//! rebooted the board; kws-x1-init fixes; TX SOP fix; PING-vs-
//! Get_Source_Cap (type 5 vs 7); RDO bit 25; carrier-idle RX gate;
//! re-attach pulse. See git log for the full chain.)

#![no_std]
#![no_main]

mod board;
mod buttons;
mod lcd;
mod pd;
mod pin_trace;
mod ui;

static TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// EXINT line bits latched by the ISRs (bit n = line n saw an edge).
static EXTI_LATCH: AtomicU32 = AtomicU32::new(0);
/// FUSB302 INT_N (PC13, line 13) edge count.
static INT_COUNT: AtomicU32 = AtomicU32::new(0);
/// ISR -> main-loop doorbell (coalescing-safe: the latch carries the bits).
static DOORBELL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Build tag, carried on every [meter] console line so a paste can
/// never be mistaken for another build (the boot banner is usually lost
/// before the host opens the port).
const BUILD: &[u8] = b"r111";

use crate::ui::Press;
use crate::ui::push_pdo;
use crate::ui::Screen;
use crate::ui::Ui;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Timer;
use embassy_time::with_timeout;
// The cortex_m_rt::interrupt attribute macro expands references to the
// device's `interrupt` module — bring it into scope for the handlers.
use embassy_at32::pac::at32f403a::interrupt;

// --- Vendor C stack glue (compiled by build.rs into libvendorusb.a) ---

unsafe extern "C" {
    fn fnb58_usb_init();
    fn fnb58_usb_recv(buffer: *mut u8) -> u16;
    fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
    fn fnb58_usb_configured() -> u32;
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
    say(b"[lcd] FNB58 r111: rust lcd\r\n").await;

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

    // --- r111: LCD on the Rust embassy HAL (SPI2 + GPIO) ---
    // Lcd::new() performs the SWJTAG remap, so it must come before the
    // button pins (PB4/PA15) are configured.
    let mut lcd = unsafe { lcd::Lcd::new() };
    lcd.init().await;
    say(b"[lcd] init done, drawing bars\r\n").await;
    lcd.test_pattern().await;
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

    // --- UI: POWER + PD screens; buttons navigate (r70) ---
    // Buttons: to-GND switches, internal pull-ups latched at init
    // (README pin map: LEFT=PA15, OK=PB4, BACK=PB9, RIGHT=PB8).
    let mut btns = buttons::Buttons::new();
    let mut sender = Sender::new();
    let mut ui = Ui::new();
    ui.enter(&mut lcd, Screen::Power).await;

    // r102: EXINT lines. Buttons keep their pull-ups (Buttons::new) and
    // gain edge interrupts (both edges — the loop level-debounces as
    // before, but a press now WAKES it). PC13 = the user-traced FUSB302
    // INT_N: open-drain, so an internal pull-up, falling edge.
    let _btn_exti_left = unsafe {
        embassy_at32::exti::ExtiInput::new(
            embassy_at32::gpio::Port::A,
            15,
            embassy_at32::exti::Trigger::Both,
        )
    }
    .expect("PA15 exti");
    let _btn_exti_ok = unsafe {
        embassy_at32::exti::ExtiInput::new(
            embassy_at32::gpio::Port::B,
            4,
            embassy_at32::exti::Trigger::Both,
        )
    }
    .expect("PB4 exti");
    let _btn_exti_back = unsafe {
        embassy_at32::exti::ExtiInput::new(
            embassy_at32::gpio::Port::B,
            9,
            embassy_at32::exti::Trigger::Both,
        )
    }
    .expect("PB9 exti");
    let _btn_exti_right = unsafe {
        embassy_at32::exti::ExtiInput::new(
            embassy_at32::gpio::Port::B,
            8,
            embassy_at32::exti::Trigger::Both,
        )
    }
    .expect("PB8 exti");
    let pc13 =
        unsafe { embassy_at32::gpio::InputPin::new_pull_up(embassy_at32::gpio::Port::C, 13) }
            .expect("PC13 claim");
    let _pd_int =
        unsafe { embassy_at32::exti::ExtiInput::from_input(pc13, embassy_at32::exti::Trigger::Falling) };
    unsafe {
        use embassy_at32::pac::at32f403a::interrupt;
        cortex_m::peripheral::NVIC::unmask(interrupt::EXINT4);
        cortex_m::peripheral::NVIC::unmask(interrupt::EXINT9_5);
        cortex_m::peripheral::NVIC::unmask(interrupt::EXINT15_10);
    }

    // r88: no FUSB302 init at boot. `pd_on` gates EVERY pd:: call — the
    // controller powers up only on the first explicit scan (OK on the PD
    // screen); from then on the poll keeps the sink contract alive.
    let mut caps = pd::SourceCaps::default();
    let mut listener = pd::Listener::default();
    let mut pd_on = false;
    // r93: the line each scan transmits/receives on — starts on the line
    // init() measured, then every OK press flips to the other one. The
    // r91/r92 logs (self-TX heard on CC1 with both drivers armed,
    // silence on CC2 with TXCC2 alone) make the TX path the open
    // question; alternating answers it in one session.
    let mut scan_cc = 0u8;

    let mut tick: u32 = 0;
    // LEXT probe: enable the 32.768 kHz crystal oscillator on PC14/PC15
    // and wait for its stable flag. If the board has the RTC crystal
    // footprint populated, lextstblf sets within a second; with nothing
    // fitted it never does.
    let lext_ok = {
        let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
        crm.bpdc().modify(|_, w| w.lexten().set_bit());
        let mut ok = false;
        for _ in 0..2000 {
            if crm.clkint().read().lextstblf().bit() {
                ok = true;
                break;
            }
            cortex_m::asm::delay(192000); // ~1 ms at 192 MHz
        }
        ok
    };

    // Reset-cause flags (CRM CTRLSTS bits 26..=31, sticky until cleared):
    // 0x01 pin, 0x02 POR/LVR, 0x04 software, 0x08 WDT, 0x10 WWT, 0x20 LP.
    let rst_flags = {
        let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
        let flags = ((crm.ctrlsts().read().bits() >> 26) & 0x3F) as u8;
        // Clear the flags so the NEXT boot reports only its own fresh
        // cause (the bits are sticky and would otherwise accumulate).
        crm.ctrlsts().write(|w| {
            w.licken().set_bit(); // keep the LSI enabled as before
            w.rstfc().set_bit()  // clear all reset-cause flags
        });
        flags
    };
    ui.draw_rst(&mut lcd, rst_flags).await;
    {
        let mut d = [0u8; 64];
        let mut n = 0;
        for &b in b"[boot] rst=" {
            d[n] = b;
            n += 1;
        }
        const HEX: &[u8; 16] = b"0123456789abcdef";
        d[n] = HEX[(rst_flags >> 4) as usize];
        d[n + 1] = HEX[(rst_flags & 0xF) as usize];
        n += 2;
        for &b in b" loop start lext=" {
            d[n] = b;
            n += 1;
        }
        d[n] = b'0' + lext_ok as u8;
        n += 1;
        d[n] = b'\r';
        d[n + 1] = b'\n';
        sender.push(&d[..n + 2]);
    }

    // One-time ISR proof per EXINT line (the buttons double as the
    // interrupt canary; line 13 = the FUSB302 INT_N).
    let mut isr_seen = [false; 16];

    loop {
        // Wake on ANY EXTI edge (buttons, FUSB302 INT_N) or every 10 ms
        // as the fallback cadence. The ISR latches line bits and rings
        // the doorbell; nothing is lost when several edges coalesce.
        let _ = with_timeout(Duration::from_millis(10), DOORBELL.wait()).await;
        let events = EXTI_LATCH.swap(0, Ordering::AcqRel);
        tick = tick.wrapping_add(1);

        for line in [4u8, 8, 9, 13, 15] {
            if !isr_seen[line as usize] && events & (1u32 << line) != 0 {
                isr_seen[line as usize] = true;
                let mut d = [0u8; 32];
                let mut n = 0;
                for &b in b"[exti] isr line=" {
                    d[n] = b;
                    n += 1;
                }
                n = push_dec(&mut d, n, line as u32);
                for &b in b" ok\r\n" {
                    d[n] = b;
                    n += 1;
                }
                sender.push(&d[..n]);
            }
        }
        if events & (1u32 << 13) != 0 {
            INT_COUNT.fetch_add(1, Ordering::Relaxed);
        }

        // PD poll — every iteration (the 10 ms fallback cadence), plus
        // immediately after each FUSB302 INT_N edge: the interrupt wake
        // is what makes the Request meet tSenderResponse reliably.
        if pd_on {
            if matches!(pd::poll(&mut tpa, &mut caps, &mut listener).await, Ok(true)) {
                report_caps(&caps, &mut sender);
                report_request(&mut sender, &listener);
            }
            if ui.screen == Screen::Pd {
                ui.draw_pd(&mut lcd, &caps).await;
            }
        }

        // Buttons: debounced active-low (50 ms at this cadence); events
        // to the console queue, presses drive the UI screens.
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
                Press::Switch(screen) => ui.enter(&mut lcd, screen).await,
                Press::Discover => {
                    // r104: re-init on EVERY scan. The r103 log showed
                    // the first scan after a cold boot works end-to-end
                    // (fresh FUSB302 state) while every later scan on
                    // the same chip state fails to transmit (r=2: no
                    // TXSENT, no RETRYFAIL — the transmitter never
                    // starts). The full reset + config replicates the
                    // working state each press.
                    match pd::init(&mut tpa).await {
                        Ok((id, cc)) => {
                            pd_on = true;
                            scan_cc = cc;
                            listener.set_cc(cc);
                            let st = pd::status(&mut tpa).await;
                                let mut d = [0u8; 64];
                                let mut n = 0;
                                for &b in b"[pd] fusb302 id=" {
                                    d[n] = b;
                                    n += 1;
                                }
                                n = push_hex8(&mut d, n, id as u32);
                                for &b in b" cc=" {
                                    d[n] = b;
                                    n += 1;
                                }
                                d[n] = b'0' + cc;
                                n += 1;
                                for &b in b" st=" {
                                    d[n] = b;
                                    n += 1;
                                }
                                if let Ok((st0, st1)) = st {
                                    n = push_hex8(&mut d, n, st0 as u32);
                                    d[n] = b'/';
                                    n += 1;
                                    n = push_hex8(&mut d, n, st1 as u32);
                                } else {
                                    for &b in b"ERR" {
                                        d[n] = b;
                                        n += 1;
                                    }
                                }
                                d[n] = b'\r';
                                d[n + 1] = b'\n';
                                sender.push(&d[..n + 2]);
                        }
                        Err(_) => sender.push(b"[pd] init FAIL\r\n"),
                    }
                    if pd_on {
                        // r107: discover() itself re-measures the
                        // orientation, routes (incl. the GoodCRC spec
                        // revision) and soft-resets before asking.
                        let acked = match pd::discover(&mut tpa, &mut caps, &mut listener).await {
                            Ok(true) => {
                                report_caps(&caps, &mut sender);
                                report_request(&mut sender, &listener);
                                sender.push(b"[pd] contract requested\r\n");
                                true
                            }
                            Ok(false) => false,
                            Err(_) => false,
                        };
                        // Per-scan evidence on ONE line (the pd=1 diag
                        // lines tend to scroll away): soft-reset ack, TX
                        // outcome, ever-acked-incoming, and a raw dump of
                        // whatever still sits in the RX FIFO.
                        let mut d = [0u8; 96];
                        let mut n = 0;
                        for &b in b"[pd] scan cc=" {
                            d[n] = b;
                            n += 1;
                        }
                        d[n] = b'0' + scan_cc;
                        n += 1;
                        // BC_LVL of both lines at the scan's orientation
                        // measure (r103: high nibble CC1, low CC2; 0 = no
                        // Rp on that line — the port state in one byte).
                        let mm = listener.meas();
                        for &b in b" m=" {
                            d[n] = b;
                            n += 1;
                        }
                        n = push_hex8(&mut d, n, ((mm[0] << 4) | mm[1]) as u32);
                        for &b in b" r=" {
                            d[n] = b;
                            n += 1;
                        }
                        n = push_dec(&mut d, n, listener.last_tx_code() as u32);
                        for &b in b" g=" {
                            d[n] = b;
                            n += 1;
                        }
                        d[n] = b'0' + listener.gcrc_seen() as u8;
                        n += 1;
                        for &b in b" x=" {
                            d[n] = b;
                            n += 1;
                        }
                        d[n] = b'0' + listener.collided() as u8;
                        n += 1;
                        if !acked {
                            for &b in b" f=" {
                                d[n] = b;
                                n += 1;
                            }
                            n += pd::rx_dump(&mut tpa, &mut d[n..]).await;
                        }
                        if acked {
                            for &b in b" ACKED" {
                                d[n] = b;
                                n += 1;
                            }
                        }
                        d[n] = b'\r';
                        d[n + 1] = b'\n';
                        sender.push(&d[..n + 2]);
                    }
                }
                Press::None => {}
            }
        }

        // Meter: 200 ms cadence (screen values; TPA conversion window).
        if tick % 20 == 0 {
            let ok = match (
                tpa_read(&mut tpa, 0x02).await,
                tpa_read(&mut tpa, 0x04).await,
                tpa_read(&mut tpa, 0x03).await,
            ) {
                (Ok(bus), Ok(cur), Ok(pwr)) => {
                    let mv = (bus as u32 * 5) / 4; // 1.25 mV/LSB
                    let ua = (cur as i16 as i32) * 50; // 50 uA/LSB
                    let mw = ((pwr as i16 as i32) * 5) / 4; // 1.25 mW/LSB
                    if ui.screen == Screen::Power {
                        ui.draw_power(&mut lcd, mv, ua, mw).await;
                    }
                    true
                }
                _ => false,
            };
            let _ = ok;
        }

        // Periodic diag heartbeat with the build tag, queue depth, the
        // send-failure count (f>0 = the host is not collecting packets)
        // and the PD power state (pd=1 adds the per-CC receiver diag).
        if tick % 200 == 0 {
            let mut d = [0u8; 96];
            let mut n = 0;
            for &b in b"[diag " {
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
            for &b in b" i=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, INT_COUNT.load(Ordering::Relaxed) % 100000);
            for &b in b" pd=" {
                d[n] = b;
                n += 1;
            }
            d[n] = b'0' + pd_on as u8;
            n += 1;
            if pd_on {
                n = listener.write_diag(&mut d, n);
            } else {
                d[n] = b'\r';
                d[n + 1] = b'\n';
                n += 2;
            }
            sender.push(&d[..n]);
        }

        sender.drain();
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

/// Report the auto-Request's transmission outcome after a caps capture
/// (0 = acked — the contract should stand; 1 = retryfail; 2 = timeout),
/// plus the r108 wedge evidence: STATUS1 after the attempt (TX_FULL =
/// tokens never consumed; TX_EMPTY = consumed, protocol layer silent)
/// and the raw INTERRUPTA byte.
fn report_request(out: &mut Sender, lis: &pd::Listener) {
    let mut d = [0u8; 48];
    let mut n = 0;
    for &b in b"[pd] request r=" {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, lis.last_tx_code() as u32);
    for &b in b" s1=" {
        d[n] = b;
        n += 1;
    }
    n = push_hex8(&mut d, n, lis.st1() as u32);
    for &b in b" ia=" {
        d[n] = b;
        n += 1;
    }
    n = push_hex8(&mut d, n, lis.ia() as u32);
    d[n] = b'\r';
    d[n + 1] = b'\n';
    out.push(&d[..n + 2]);
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

/// EXINT ISR body: clear the hardware pending flags, latch the line
/// bits, ring the main-loop doorbell.
fn exti_isr(lo: u8, hi: u8) {
    embassy_at32::exti::on_interrupts_in(lo..=hi);
    let mut bits = 0u32;
    for line in lo..=hi {
        if embassy_at32::exti::take_edge(line) {
            bits |= 1u32 << line;
        }
    }
    if bits != 0 {
        EXTI_LATCH.fetch_or(bits, Ordering::AcqRel);
        DOORBELL.signal(());
    }
}

#[cortex_m_rt::interrupt]
unsafe fn EXINT4() {
    exti_isr(4, 4);
}

#[cortex_m_rt::interrupt]
unsafe fn EXINT9_5() {
    exti_isr(5, 9);
}

#[cortex_m_rt::interrupt]
unsafe fn EXINT15_10() {
    exti_isr(10, 15);
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
