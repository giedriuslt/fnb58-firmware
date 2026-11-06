//! FNB58 r139: PD first, buffered boot log, stall watch. The FUSB302
//! session starts at boot on PC power — USB configured within a BOUNDED
//! 3 s (r138's ungated boot session reboot-looped charger-only power:
//! the session's auto-GoodCRCs string the source along, ITS Hard Reset
//! dips VBUS, brownout, repeat; the log itself never waits — the 4096-B
//! BootLog ring below is flushed when the host connects). A failed
//! session now restores the FUSB302 power-on baseline so the source is
//! not left waiting for a Request. r138's PC symptom (console stops
//! right after the ring flush, no heartbeat) is instrumented: pd_task
//! logs its lifecycle through PD_LOG, both tasks stamp stage atomics,
//! and the SysTick stall watch prints '[watch] main=N pd=M' from the
//! exception if the main loop stops moving; panics dump their location.
//! PC13 (FUSB302 INT_N) EXINT goes live before the PD task spawns and
//! pd_task's duplicated second CC diagnostic is gone (~640 ms earlier).
//! (History r88-r137: EXINT r102; kws flow r119; safe mode r120;
//! buzzer/accel/flash-read/console-cmds r121-r133; PC link r134/r135;
//! internal temp r136/r137; early-PD + BootLog r138. See git log.)

#![no_std]
#![no_main]

mod board;
mod adc;
mod pd_phy;
mod pd_task;
mod buttons;
mod buzzer;
mod flash;
mod kxtj3;
mod lcd;
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
const BUILD: &[u8] = b"r161";

use crate::ui::Press;
use crate::ui::Screen;
use crate::ui::Ui;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU8;
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
    // r134: PC-link mode (fnb58_hid.c) — the stock FNB58 USB device.
    fn fnb58_usb_init_hid();
    fn fnb58_hid_tx(report: *const u8) -> i32;
    fn fnb58_hid_rx(buffer: *mut u8) -> u16;
}

/// Called from the vendor IRQ handlers.
#[unsafe(no_mangle)]
extern "C" fn fnb58_usbd_event_hook() {}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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

    // --- r136: internal temperature sensor (ADC1 CH16, VREFINT-corrected;
    // see adc.rs). Mode-independent: the PC-link reports and the console
    // heartbeat both read it. ---
    adc::init();

    // --- r134: PC-link mode (OpenFNB58 protocol) ---
    // Hold OK while plugging in: the device enumerates as the stock
    // FNB58 USB HID measurement interface (2E3C:5558, endpoints
    // 0x83/0x03, 64-byte reports) instead of the CDC console. The
    // console stays the default — no button, no change. OK = PB4,
    // to-GND switch with an internal pull-up.
    let pc_link = {
        use embassy_at32::gpio::{InputPin, Port};
        let ok = unsafe { InputPin::new_pull_up(Port::B, 4).expect("PB4 claim") };
        let mut held = true;
        for _ in 0..8 {
            if !ok.is_low() {
                held = false;
                break;
            }
            cortex_m::asm::delay(192 * 50); // ~50 us pull-up settle
        }
        held
    };

    if pc_link {
        // I2C1 up for the TPA626 meter only — NO PD task: the OK press
        // that selected this mode must not trigger a PD session, and
        // the PC link is a data stream, not a trigger (r120 safe mode).
        {
            use embassy_at32::gpio::{AlternatePin, Drive, Port};
            use embassy_at32::i2c::{I2c, Speed};
            let _scl = unsafe { AlternatePin::new_open_drain(Port::B, 6, Drive::Moderate) };
            let _sda = unsafe { AlternatePin::new_open_drain(Port::B, 7, Drive::Moderate) };
            let i2c = unsafe { I2c::new_i2c1(clocks.pclk1_hz, Speed::Standard) };
            let mut guard = pd_task::BUS.lock().await;
            *guard = pd_task::BusCell(i2c.ok());
        }

        // USB as the FNB58 vendor HID device (fnb58_hid.c).
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_L_CAN1_RX0);
            cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_H_CAN1_TX);
            fnb58_usb_init_hid();
        }
        for _ in 0..30 {
            if unsafe { fnb58_usb_configured() } == 1 {
                break;
            }
            Timer::after_millis(100).await;
        }

        // The mode is visible and audible on the device as well: the
        // power screen with a PC-LINK status line, a higher boot beep
        // than the console's 500 Hz.
        let mut buzzer = unsafe { buzzer::Buzzer::new() };
        let spi = unsafe {
            embassy_at32::spi::Spi::new_spi2(
                embassy_at32::spi::Mode::Mode0,
                embassy_at32::spi::Divider::Div4,
            )
        };
        let mut lcd = unsafe { lcd::Lcd::new(spi) };
        lcd.init().await;
        let mut ui = Ui::new();
        ui.enter(&mut lcd, Screen::Power).await;
        buzzer.buzz(700, 60);

        // TPA626 streaming config: no averaging, 588 us conversions —
        // every 10 ms sample is fresh (the console's power screen uses
        // AVG=64, which refreshes only ~7x/s). Same calibration.
        {
            let mut guard = pd_task::BUS.lock().await;
            if let Some(bus) = guard.0.as_mut() {
                let _ = tpa_write(bus, 0x00, 0x0337).await; // AVG=1, 588us+588us, cont
                let cal: u16 = (5_120_000u32 / 500u32) as u16; // 10 mOhm shunt, 50 uA LSB
                let _ = tpa_write(bus, 0x05, cal).await;
            }
        }

        // OpenFNB58 stream: 25 reports/s of AA 04 + 4 x 15-byte samples
        // (voltage u32/100000 V, current u32/100000 A, D+ u16/1000 V,
        // D- u16/1000 V, unknown u8, temp u16/10 C). V/A come from the
        // TPA626, temperature from the internal sensor (refreshed every
        // report); D+/D- and the unknown byte remain 0 sentinels until
        // this firmware measures them.
        let mut report = [0u8; 64];
        report[0] = 0xAA;
        report[1] = 0x04;
        // Tenths of a degree C (the report field's own unit); None reads
        // keep the previous value.
        let mut temp_deci: u16 = 0;
        let mut slot = 0usize;
        let mut lcd_tick = 0u32;
        // Command tallies, shown on the LCD status line: whether the
        // host's INIT1/INIT2/POLL reports actually reach the device.
        let (mut n_init1, mut n_init2, mut n_poll) = (0u32, 0u32, 0u32);
        let mut cmd = [0u8; 64];
        let mut ticker = embassy_time::Ticker::every(Duration::from_millis(10));
        // First paint: the mode is on screen before the host connects.
        lcd.draw_text(8, 224, b"r161 PC LINK\0", 1, 0x7BEF, 0x0000).await;
        loop {
            ticker.next().await;
            // One meter sample into the next report slot.
            let (mv, ua) = {
                let mut guard = pd_task::BUS.lock().await;
                match guard.0.as_mut() {
                    Some(bus) => {
                        match (tpa_read(bus, 0x02).await, tpa_read(bus, 0x04).await) {
                            (Ok(v), Ok(i)) => ((v as u32 * 5) / 4, (i as i16 as i32) * 50),
                            _ => (0, 0),
                        }
                    }
                    None => (0, 0),
                }
            };
            let off = 2 + slot * 15;
            report[off..off + 4].copy_from_slice(&(mv * 100).to_le_bytes()); // V * 100000
            let amps_100k = if ua > 0 { (ua / 10) as u32 } else { 0 };
            report[off + 4..off + 8].copy_from_slice(&amps_100k.to_le_bytes()); // A * 100000
            // D+ / D- (off+8..off+12) and the unknown byte (off+12):
            // 0 sentinels; the buffer starts zeroed and only the first
            // two fields are ever rewritten.
            slot += 1;
            if slot == 4 {
                slot = 0;
                // Temperature: one sensor read per report, stamped into
                // all four samples (u16 LE, tenths of a degree C).
                if let Some(t) = adc::read_temp_deci() {
                    temp_deci = t.clamp(0, 6553) as u16;
                }
                for s in 0..4 {
                    let off = 2 + s * 15;
                    report[off + 13..off + 15].copy_from_slice(&temp_deci.to_le_bytes());
                }
                // Retry for one busy cycle, then drop: at 25 Hz a lost
                // report costs 40 ms of the stream, stalling costs more.
                for _ in 0..2 {
                    if unsafe { fnb58_hid_tx(report.as_ptr()) } == 0 {
                        break;
                    }
                }
            }

            // Host commands (INIT1/INIT2/POLL/trigger). The stream runs
            // from enumeration, so every one of them finds the device
            // already doing what it asks — they are counted, not acted
            // on. CRC-8: poly 0x39, init 0x42 over bytes 1..62 vs 63.
            if unsafe { fnb58_hid_rx(cmd.as_mut_ptr()) } == 64
                && cmd[0] == 0xAA
                && crc8_fnb58(&cmd[1..63]) == cmd[63]
            {
                match cmd[1] {
                    0x81 => n_init1 += 1,
                    0x82 => n_init2 += 1,
                    0x83 => n_poll += 1,
                    _ => {}
                }
            }

            // Live LCD values, ~4 Hz, from the latest sample (the
            // protocol field scales convert straight back to mV/uA),
            // plus the PC-LINK status line: die temperature (the same
            // value the reports carry) and the host command tallies.
            // Fixed-width 31-char line (spaces then NUL at the end) at
            // scale 1 so the ~30-char worst case fits the 320px screen
            // and every overdraw erases ghosts, like ui.rs Line12::pad.
            lcd_tick += 1;
            if lcd_tick % 25 == 0 {
                let mw = (mv as i32 * ua) / 1000;
                ui.draw_power(&mut lcd, mv, ua, mw).await;
                let mut line = [b' '; 32];
                let mut w = 0;
                for &b in b"r161 " {
                    line[w] = b;
                    w += 1;
                }
                let t = temp_deci as i32;
                if t < 0 {
                    line[w] = b'-';
                    w += 1;
                }
                let ta = if t < 0 { -t } else { t };
                w = push_dec(&mut line, w, (ta / 10) as u32);
                line[w] = b'.';
                w += 1;
                line[w] = b'0' + (ta % 10) as u8;
                w += 1;
                line[w] = b'C';
                w += 1;
                for &b in b" i1=" {
                    line[w] = b;
                    w += 1;
                }
                w = push_dec(&mut line, w, n_init1 % 1000);
                for &b in b" i2=" {
                    line[w] = b;
                    w += 1;
                }
                w = push_dec(&mut line, w, n_init2 % 1000);
                for &b in b" p=" {
                    line[w] = b;
                    w += 1;
                }
                w = push_dec(&mut line, w, n_poll % 1000);
                line[31] = 0;
                lcd.draw_text(8, 224, &line, 1, 0x7BEF, 0x0000).await;
            }
        }
    }

    // --- r138: the PD session starts AT BOOT, before USB ---
    // The charger is still announcing its capabilities for ~10 s after
    // attach; the earlier the engine's passive wait begins, the more of
    // that window it catches. PC13 (FUSB302 INT_N) EXINT goes live FIRST
    // so the whole boot session's edges are counted — kws-x1 creates its
    // ExtiInput before spawning the PD task (the fusb302-pd stack polls
    // I2C, the IRQ is evidence only). Line 13 sits in EXINT15_10; PA15
    // (line 15) joins later with its own ExtiInput setup, so this early
    // unmask cannot fire it.
    let pc13 = unsafe {
        embassy_at32::gpio::InputPin::new_pull_up(embassy_at32::gpio::Port::C, 13)
    }
    .expect("PC13 claim");
    let _pd_int = unsafe {
        embassy_at32::exti::ExtiInput::from_input(pc13, embassy_at32::exti::Trigger::Falling)
    };
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::EXINT15_10);
    }
    // The I2C1 bus goes into the shared cell and the task spawns before
    // the USB bring-up (which no longer blocks: see the BootLog ring).
    {
        use embassy_at32::gpio::{AlternatePin, Drive, Port};
        use embassy_at32::i2c::{I2c, Speed};

        let _scl = unsafe { AlternatePin::new_open_drain(Port::B, 6, Drive::Moderate) };
        let _sda = unsafe { AlternatePin::new_open_drain(Port::B, 7, Drive::Moderate) };
        // r129: PB14 = SPI2_MISO for the W25Q128 flash reads (the LCD
        // never needed MISO, so this pin was never muxed before).
        let _miso = unsafe { AlternatePin::new_push_pull(Port::B, 14, Drive::Moderate) };
        // r150: back to 100 kHz Standard — r148's 400 kHz made the FUSB302
        // (0x22) fail its transactions outright (start err=4 Config,
        // baseline restore FAIL, err=768 on every press; the TPA626
        // limped along) — this board's pull-ups do not support Fast mode.
        // The speed was never the issue: the real r115-era root cause was
        // the never-armed selection (r149).
        let i2c = unsafe { I2c::new_i2c1(clocks.pclk1_hz, Speed::Standard) };
        {
            let mut guard = pd_task::BUS.lock().await;
            *guard = pd_task::BusCell(i2c.ok());
        }
        spawner.spawn(pd_task::run().expect("one pd task"));
    }

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

    // r138: no USB wait. Enumeration proceeds in the USB interrupts while
    // the LCD/meter/accel bring-up runs; every boot line lands in the
    // 4096-byte boot-log ring (BootLog below) and is sent once the host
    // connects (the main loop flushes it, oldest first).
    fn say(msg: &[u8]) {
        boot_log().push(msg);
    }
    say(b"[lcd] FNB58 r161: tx timestamps\r\n");

    // (The C glue's I2C health check is gone with the vendor i2c code;
    // the Rust meter's mfr read below proves the bus the same way.)

    // --- r121: buzzer on PA8 (pin 29, user-identified) ---
    let mut buzzer = unsafe { buzzer::Buzzer::new() };

    // --- r111/r126: LCD + W25Q128 flash share SPI2 (each has its own
    // chip-select). Lcd::new() performs the SWJTAG remap (value 100: full
    // SWJ disabled — PA13/PA14 join the GPIO set; flashing is unaffected,
    // it uses the BOOT0-high USB DFU), so it must come before the flash
    // CS claim (PA13) and the button pins.
    let spi = unsafe {
        embassy_at32::spi::Spi::new_spi2(
            embassy_at32::spi::Mode::Mode0,
            embassy_at32::spi::Divider::Div4,
        )
    };
    let mut lcd = unsafe { lcd::Lcd::new(spi) };
    let mut flash = unsafe { flash::Flash::new(spi) };
    lcd.init().await;
    say(b"[lcd] init done, drawing bars\r\n");
    lcd.test_pattern().await;
    say(b"[lcd] pattern drawn\r\n");
    // r121: boot beep — audible confirmation of the PA8 buzzer (pin 29).
    // r138: dwell cut 1500 -> 300 ms; the loop (and so the boot-log flush)
    // starts that much sooner, and ui.enter below redraws anyway.
    buzzer.buzz(500, 30);
    Timer::after_millis(300).await;

    // --- the meter leases the shared I2C1 bus per read (r113) ---
    use embedded_hal_async::i2c::Operation;
    use embedded_hal_async::i2c::I2c as _;

    async fn tpa_write(bus: &mut embassy_at32::i2c::I2c, reg: u8, val: u16) -> Result<(), embassy_at32::i2c::Error> {
        let mut wr = [reg, (val >> 8) as u8, val as u8];
        bus.transaction(0x40, &mut [Operation::Write(&mut wr)]).await
    }
    async fn tpa_read(bus: &mut embassy_at32::i2c::I2c, reg: u8) -> Result<u16, embassy_at32::i2c::Error> {
        let mut reg_buf = [reg];
        let mut buf = [0u8; 2];
        bus.transaction(
            0x40,
            &mut [Operation::Write(&mut reg_buf), Operation::Read(&mut buf)],
        )
        .await?;
        Ok(((buf[0] as u16) << 8) | buf[1] as u16)
    }

    // Identify + configure the TPA626 (INA226 protocol).
    {
        let mut guard = pd_task::BUS.lock().await;
        if guard.0.as_mut().is_none() {
            say(b"[meter] no i2c bus\r\n");
        }
        if let Some(bus) = guard.0.as_mut() {
            match tpa_read(bus, 0xFE).await {
                Ok(0x5549) => say(b"[meter] rust i2c OK mfr=5549\r\n"),
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
                    say(&d[..n + 2]);
                }
                Err(_) => say(b"[meter] rust i2c read FAIL\r\n"),
            }
            let _ = tpa_write(bus, 0x00, 0x4897).await; // AVG=64, cont
            let cal: u16 = (5_120_000u32 / 500u32) as u16; // 10 mOhm shunt, 50 uA LSB
            let _ = tpa_write(bus, 0x05, cal).await;
        }
    }
    say(b"[meter] rust tpa configured\r\n");

    // --- r123: KXTJ3 accelerometer for the screen-flip detection ---
    let accel_ok = {
        let mut guard = pd_task::BUS.lock().await;
        match guard.0.as_mut() {
            Some(bus) => kxtj3::init(bus).await,
            None => false,
        }
    };
    // Boot gravity calibration: the tester rests however it rests; a
    // 180-degree flip reverses every axis's sign against this.
    let mut gravity0 = kxtj3::Gravity { x: 0, y: 0, z: 0 };
    if accel_ok {
        let mut guard = pd_task::BUS.lock().await;
        if let Some(bus) = guard.0.as_mut() {
            for _ in 0..16 {
                if let Ok(g) = kxtj3::read(bus).await {
                    gravity0.x += g.x / 16;
                    gravity0.y += g.y / 16;
                    gravity0.z += g.z / 16;
                }
                Timer::after_millis(25).await;
            }
        }
        let mut d = [0u8; 80];
        let mut n = 0;
        for &b in b"[accel] kxtj3 ok g0=" {
            d[n] = b;
            n += 1;
        }
        for v in [gravity0.x, gravity0.y, gravity0.z] {
            let u = v as u16;
            d[n] = b' ';
            n += 1;
            n = push_hex8(&mut d, n, u32::from(u >> 8));
            n = push_hex8(&mut d, n, u32::from(u & 0xFF));
        }
        d[n] = b'\r';
        d[n + 1] = b'\n';
        n += 2;
        say(&d[..n]);
    } else {
        say(b"[accel] kxtj3 FAIL\r\n");
    }

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
    // PC13 (FUSB302 INT_N) was claimed and EXINT15_10 unmasked before the
    // PD task spawned (r138); here the button lines join their IRQs.
    unsafe {
        use embassy_at32::pac::at32f403a::interrupt;
        cortex_m::peripheral::NVIC::unmask(interrupt::EXINT4);
        cortex_m::peripheral::NVIC::unmask(interrupt::EXINT9_5);
    }

    // r113: PD runs in its own task on the vendored fusb302-pd stack.
    // The main loop only raises the START signal (OK on the PD screen),
    // keeps the pressed-button bitmask current (BACK stops the session),
    // and renders pd_task::VIEW.

    let mut tick: u32 = 0;
    // LEXT probe: enable the 32.768 kHz crystal oscillator on PC14/PC15
    // and wait for its stable flag. If the board has the RTC crystal
    // footprint populated, lextstblf sets within a second; with nothing
    // fitted it never does.
    let lext_ok = {
        let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
        let pwc = unsafe { embassy_at32::pac::at32f403a::PWC::steal() };
        // r126: BPDC sits in the battery-backed domain — writes are
        // ignored until the PWC clock is on and PWC.CTRL.BPWEN is set
        // (vendor pwc_battery_powered_domain_access). The r86 probe
        // skipped this, so LEXTEN never actually set.
        crm.apb1en().modify(|_, w| w.pwc().bit(true));
        pwc.ctrl().modify(|_, w| w.bpwen().set_bit());
        crm.bpdc().modify(|_, w| w.lexten().set_bit());
        let mut ok = false;
        for _ in 0..2000 {
            // The stable flag lives in BPDC.LEXTSTBL (bit 1); the
            // CLKINT.LEXTSTBLF polled before is the interrupt flag and
            // only sets with its interrupt enable on — a false negative
            // in every round since r86.
            if crm.bpdc().read().lextstbl().bit() {
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
    // USB command line buffer (r130): PuTTY keys arrive one packet at a
    // time; commands run on Enter (CR/LF).
    let mut cmd_buf = [0u8; 12];
    let mut cmd_len = 0;
    // Screen orientation (r122): the APPLIED state and the detector's
    // settled state (r125: the hold tracks the detected state, so the
    // un-flip can fire too — r124 only ever flipped once).
    let mut flipped = false;
    let mut last_seen = false;
    let mut flip_hold: u32 = 0;
    // Held-button bitmask for the PD session's stop gate (persists across
    // loop iterations; r114 reset it every pass, breaking BACK-stop).
    let mut held: u8 = 0;

    loop {
        // Wake on ANY EXTI edge (buttons, FUSB302 INT_N) or every 10 ms
        // as the fallback cadence. The ISR latches line bits and rings
        // the doorbell; nothing is lost when several edges coalesce.
        MAIN_STAGE.store(1, Ordering::Relaxed); // stall watch: doorbell
        let _ = with_timeout(Duration::from_millis(10), DOORBELL.wait()).await;
        let events = EXTI_LATCH.swap(0, Ordering::AcqRel);
        MAIN_STAGE.store(2, Ordering::Relaxed); // loop body
        MAIN_SEQ.fetch_add(1, Ordering::Relaxed);
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

        // r113: the PD screen renders the engine view; the session runs
        // in its own task on the shared bus. r160: NOT during a session —
        // the blocking SPI draw starves the executor (the engine's
        // transmit chain) past the charger's caps->HR window.
        if ui.screen == Screen::Pd && !pd_task::SESSION_ACTIVE.load(Ordering::Relaxed) {
            ui.draw_pd(&mut lcd, &pd_task::VIEW).await;
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

            if down {
                held |= 1 << i;
                // Click feedback: 500 Hz for 30 ms (r122, user-tuned).
                buzzer.buzz(500, 30);
            } else {
                held &= !(1 << i);
            }
            if !down {
                continue;
            }
            match ui.on_press(i) {
                Press::Switch(screen) => ui.enter(&mut lcd, screen).await,
                Press::Discover => {
                    // r119: the kws-x1 restart - the running session stops
                    // at its next idle point, the charger is restarted with
                    // a Hard Reset, and the firmware reboots into a fresh
                    // boot session against the fresh attach.
                    sender.push(b"[pd] restart requested\r\n");
                    pd_task::request_restart();
                }
                Press::None => {}
            }
        }
        pd_task::BTN.store(held, Ordering::Relaxed);

        // Orientation poll: 200 ms cadence; a flip must hold ~1 s before
        // the screen rotates (kws-x1's settle approach). r160: not during
        // a PD session (I2C bus contention with the engine's chain).
        if accel_ok && !pd_task::SESSION_ACTIVE.load(Ordering::Relaxed) && tick % 20 == 0 {
            MAIN_STAGE.store(3, Ordering::Relaxed); // accel BUS lock
            let mut guard = pd_task::BUS.lock().await;
            if let Some(bus) = guard.0.as_mut() {
                if let Ok(g) = kxtj3::read(bus).await {
                    // r124: "flipped" = the current gravity points mostly
                    // OPPOSITE the boot calibration (dot product strongly
                    // negative): the device was turned over ~180 degrees,
                    // independent of which axis the board mounts the
                    // sensor along. Settled = the current vector is as
                    // large as the calibration (not mid-tilt).
                    let dot = i32::from(g.x) * i32::from(gravity0.x)
                        + i32::from(g.y) * i32::from(gravity0.y)
                        + i32::from(g.z) * i32::from(gravity0.z);
                    let norm0 = i32::from(gravity0.x) * i32::from(gravity0.x)
                        + i32::from(gravity0.y) * i32::from(gravity0.y)
                        + i32::from(gravity0.z) * i32::from(gravity0.z);
                    let norm = i32::from(g.x) * i32::from(g.x)
                        + i32::from(g.y) * i32::from(g.y)
                        + i32::from(g.z) * i32::from(g.z);
                    let now_flipped = 4 * dot < -norm0 && norm * 4 >= norm0;
                    // The hold counts consecutive polls agreeing with the
                    // detected state; when it has settled for ~1 s and the
                    // settled state differs from the APPLIED one, apply it
                    // (works for the flip AND the un-flip).
                    if now_flipped == last_seen {
                        flip_hold += 1;
                    } else {
                        flip_hold = 0;
                        last_seen = now_flipped;
                    }
                    if flip_hold >= 5 && flipped != now_flipped {
                        flipped = now_flipped;
                        lcd.set_inverted(flipped).await;
                        // Force a full screen refresh (the chrome moved).
                        ui.enter(&mut lcd, ui.screen).await;
                    }
                }
            }
        }

        // Meter: 200 ms cadence (screen values; TPA conversion window).
        // r160: not during a PD session (bus contention + the draw).
        if !pd_task::SESSION_ACTIVE.load(Ordering::Relaxed) && tick % 20 == 0 {
            MAIN_STAGE.store(4, Ordering::Relaxed); // meter BUS lock
            let mut guard = pd_task::BUS.lock().await;
            let Some(bus) = guard.0.as_mut() else {
                continue;
            };
            let ok = match (
                tpa_read(bus, 0x02).await,
                tpa_read(bus, 0x04).await,
                tpa_read(bus, 0x03).await,
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

        // USB serial commands (r130): 'i' + Enter = JEDEC ID,
        // 'f<sector>' + Enter = dump one 4 KiB flash sector as hex.
        // READ-ONLY by design — the flash driver has no write/erase code
        // at all. PuTTY sends each keystroke as its own packet: commands
        // accumulate in a line buffer and run on Enter (CR/LF). Every
        // completed command is ECHOED first, so a missing response is
        // attributable: echo seen = the bytes arrived and parsed; no
        // echo = they did not.
        {
            let mut rbuf = [0u8; 64];
            let rn = unsafe { fnb58_usb_recv(rbuf.as_mut_ptr()) };
            for &b in &rbuf[..rn.min(64) as usize] {
                if b == b'\r' || b == b'\n' {
                    if cmd_len > 0 {
                        let mut d = [0u8; 40];
                        let mut w = 0;
                        for &b in b"[flash] cmd:" {
                            d[w] = b;
                            w += 1;
                        }
                        for &b2 in &cmd_buf[..cmd_len] {
                            d[w] = b' ';
                            w += 1;
                            d[w] = b2;
                            w += 1;
                        }
                        d[w] = b'\r';
                        d[w + 1] = b'\n';
                        w += 2;
                        send_direct(&d[..w]);
                        run_flash_command(&mut flash, &cmd_buf[..cmd_len], &mut sender);
                        cmd_len = 0;
                    }
                } else if cmd_len < cmd_buf.len() {
                    cmd_buf[cmd_len] = b;
                    cmd_len += 1;
                }
            }
        }

        // Periodic diag heartbeat with the build tag, queue depth, the
        // send-failure count (f>0 = the host is not collecting packets)
        // and the PD power state (pd=1 adds the per-CC receiver diag).
        if tick % 200 == 0 {
            // r140: 128-wide so every field's worst case fits (the old 96
            // could overflow at 5-digit rx_packets / negative temps).
            let mut d = [0u8; 128];
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
            n = push_dec(&mut d, n, sender.take_fails() % 10_000);
            for &b in b" i=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, INT_COUNT.load(Ordering::Relaxed) % 100000);
            for &b in b" pd=" {
                d[n] = b;
                n += 1;
            }
            for &b in b" ph=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(pd_task::VIEW.phase.load(Ordering::Relaxed)));
            for &b in b" err=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(pd_task::VIEW.error.load(Ordering::Relaxed)));
            for &b in b" cc=" {
                d[n] = b;
                n += 1;
            }
            let cc1 = pd_task::VIEW.cc1.load(Ordering::Relaxed);
            let cc2 = pd_task::VIEW.cc2.load(Ordering::Relaxed);
            n = push_hex8(&mut d, n, (u32::from(cc1) << 4) | u32::from(cc2));
            for &b in b" li=" {
                d[n] = b;
                n += 1;
            }
            d[n] = b'0' + pd_task::VIEW.listen.load(Ordering::Relaxed);
            n += 1;
            for &b in b" st=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(pd_task::VIEW.boot_step.load(Ordering::Relaxed)));
            d[n] = b'/';
            n += 1;
            d[n] = b'0' + u8::from(pd_task::VIEW.tx_written.load(Ordering::Relaxed));
            n += 1;
            d[n] = b'/';
            n += 1;
            n = push_dec(&mut d, n, u32::from(pd_task::VIEW.rx_packets.load(Ordering::Relaxed)));
            for &b in b" rxe=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(pd_task::VIEW.rx_error.load(Ordering::Relaxed)));
            // r136: die temperature from the internal sensor (the same
            // reading the PC-link reports carry).
            for &b in b" T=" {
                d[n] = b;
                n += 1;
            }
            match adc::read_temp_deci() {
                Some(t) => {
                    if t < 0 {
                        d[n] = b'-';
                        n += 1;
                    }
                    let ta = if t < 0 { -t } else { t };
                    n = push_dec(&mut d, n, (ta / 10) as u32);
                    d[n] = b'.';
                    d[n + 1] = b'0' + (ta % 10) as u8;
                    n += 2;
                }
                None => {
                    d[n] = b'-';
                    d[n + 1] = b'-';
                    n += 2;
                }
            }
            d[n] = b'\r';
            d[n + 1] = b'\n';
            n += 2;
            sender.push(&d[..n]);
        }

        // r139: pd_task console lines join the queue (behind the boot-ring
        // flush gate, like everything else). r157: 96-wide to match the
        // widened PD_LOG (the engine-end line with TX-timeout evidence is
        // ~78 bytes; at 64 it was silently dropped at push).
        {
            let mut line = [0u8; 96];
            loop {
                let taken = PD_LOG.lock(|cell| {
                    let mut log = cell.borrow_mut();
                    if log.count == 0 {
                        return None;
                    }
                    let slot = log.head;
                    let len = log.lens[slot] as usize;
                    line.copy_from_slice(&log.lines[slot]);
                    log.head = (log.head + 1) % log.lines.len();
                    log.count -= 1;
                    Some(len)
                });
                match taken {
                    Some(len) => sender.push(&line[..len]),
                    None => break,
                }
            }
        }

        // r138: once the host connects, the boot ring is flushed first
        // (bounded chunks per tick; a busy endpoint retries next tick);
        // live queue lines follow, so the console reads in boot order.
        if boot_log().flush_step(unsafe { fnb58_usb_configured() } == 1) {
            sender.drain();
        }
    }
}

/// Run one assembled console command: 'i' = JEDEC ID, 'f<sect>' =
/// one 4 KiB sector hex dump. Unknown commands are ignored.
fn run_flash_command(flash: &mut flash::Flash, cmd: &[u8], sender: &mut Sender) {
    if cmd.is_empty() {
        return;
    }
    match cmd[0] {
        b'i' => {
            let mut d = [0u8; 32];
            let mut n = 0;
            for &b in b"[flash] jedec:" {
                d[n] = b;
                n += 1;
            }
            match flash.read_jedec_id() {
                Ok(id) => {
                    for &b in &id {
                        d[n] = b' ';
                        n += 1;
                        n = push_hex8(&mut d, n, u32::from(b));
                    }
                    d[n] = b'\r';
                    d[n + 1] = b'\n';
                    n += 2;
                    send_direct(&d[..n]);
                }
                Err(_) => sender.push(b"[flash] jedec FAIL
"),
            }
        }
        b'f' => {
            let mut sector: u32 = 0;
            let mut any = false;
            for &b in &cmd[1..] {
                if b.is_ascii_digit() {
                    sector = sector * 10 + u32::from(b - b'0');
                    any = true;
                } else {
                    break;
                }
            }
            if any && sector < 4096 {
                send_sector_dump(flash, sector);
            } else {
                sender.push(b"[flash] sector out of range (0..4095)
");
            }
        }
        _ => {}
    }
}

/// Send one line directly, retrying while the previous USB packet is
/// still in flight (bounded).
fn send_direct(line: &[u8]) {
    for _ in 0..50_000 {
        if unsafe { fnb58_usb_send(line.as_ptr(), line.len() as u16) } == 0 {
            return;
        }
        cortex_m::asm::delay(2000);
    }
}

/// Dump one 4 KiB flash sector as hex lines (16 bytes per line, direct
/// sends). READ-ONLY: the flash driver has no write/erase code at all.
fn send_sector_dump(flash: &mut flash::Flash, sector: u32) {
    static mut SECTOR: [u8; flash::SECTOR_SIZE] = [0; flash::SECTOR_SIZE];
    // r133: the header buffer was 16 bytes while "[flash] sector 1\r\n"
    // alone is 18 — a bounds panic that killed the device on every
    // f-command before the queued echo could reach the wire.
    let mut d = [0u8; 32];
    let mut n = 0;
    for &b in b"[flash] sector " {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, sector);
    d[n] = b'\r';
    d[n + 1] = b'\n';
    n += 2;
    send_direct(&d[..n]);

    // SAFETY: single task; only this command handler touches the buffer.
    let sector_buf = unsafe { &mut *core::ptr::addr_of_mut!(SECTOR) };
    if flash.read_sector(sector, sector_buf).is_err() {
        send_direct(b"[flash] read FAIL\r\n");
        return;
    }
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    // 16 bytes per line: 8 (addr+colon) + 48 (hex) = 56, inside the
    // 64-byte USB packet AND the line buffer (r126's 24-byte lines
    // overflowed the 64-byte buffer at the 19th byte -> panic).
    let mut line = [0u8; 64];
    for (li, chunk) in sector_buf.chunks(16).enumerate() {
        let mut n = 0;
        let addr = (sector as usize * flash::SECTOR_SIZE) + li * 16;
        // 6-hex-digit address (no alloc: format! would need it).
        for shift in [20u8, 16, 12, 8, 4, 0] {
            line[n] = HEX[((addr as u32) >> shift & 0xF) as usize];
            n += 1;
        }
        line[n] = b':';
        n += 1;
        for &b in chunk {
            line[n] = b' ';
            n += 1;
            n = push_hex8(&mut line, n, u32::from(b));
        }
        line[n] = b'\r';
        line[n + 1] = b'\n';
        n += 2;
        send_direct(&line[..n]);
    }
}

/// r138: 4096-byte boot-log ring. USB CDC connects seconds after boot (or
/// never, on charger-only power); every boot line is kept here and sent
/// once the host connects. Overflow drops the OLDEST bytes: recent
/// evidence wins over boot-start evidence.
pub(crate) struct BootLog {
    buf: [u8; 4096],
    /// Index of the oldest buffered byte.
    head: usize,
    /// Bytes currently buffered.
    len: usize,
    /// The ring was fully sent after connection; later lines go direct.
    flushed: bool,
    /// Oldest bytes dropped to fit newer lines.
    dropped: u32,
}

static mut BOOT_LOG: BootLog = BootLog::new();

/// The boot log. SAFETY: single-task use only — `say` during bring-up and
/// the flush step in the main loop, both on the main task.
fn boot_log() -> &'static mut BootLog {
    unsafe { &mut *core::ptr::addr_of_mut!(BOOT_LOG) }
}

impl BootLog {
    const fn new() -> Self {
        Self {
            buf: [0; 4096],
            head: 0,
            len: 0,
            flushed: false,
            dropped: 0,
        }
    }

    /// Buffer `data` (or send it directly once the ring was flushed).
    fn push(&mut self, data: &[u8]) {
        if self.flushed {
            send_direct(data);
            return;
        }
        let mut data = data;
        if data.len() > self.buf.len() {
            // Oversize line even with an empty ring: keep its tail.
            self.dropped += (data.len() - self.buf.len()) as u32;
            data = &data[data.len() - self.buf.len()..];
        }
        if self.len + data.len() > self.buf.len() {
            let drop_n = self.len + data.len() - self.buf.len();
            self.head = (self.head + drop_n) % self.buf.len();
            self.len -= drop_n;
            self.dropped += drop_n as u32;
        }
        for &b in data {
            self.buf[(self.head + self.len) % self.buf.len()] = b;
            self.len += 1;
        }
    }

    /// One non-blocking drain step once the host may be connected: send up
    /// to four 64-byte chunks, stopping to retry on a busy endpoint (the
    /// next loop tick continues). Returns true once the ring is empty and
    /// the completion note was sent; live Sender lines may then follow.
    fn flush_step(&mut self, configured: bool) -> bool {
        if self.flushed {
            return true;
        }
        if !configured {
            return false;
        }
        let mut chunk = [0u8; 64];
        for _ in 0..4 {
            if self.len == 0 {
                let mut note = [0u8; 48];
                let mut n = 0;
                for &b in b"[log] boot ring sent drop=" {
                    note[n] = b;
                    n += 1;
                }
                n = push_dec(&mut note, n, self.dropped);
                note[n] = b'\r';
                note[n + 1] = b'\n';
                n += 2;
                send_direct(&note[..n]);
                self.flushed = true;
                return true;
            }
            let take = self.len.min(64);
            let first = take.min(self.buf.len() - self.head);
            chunk[..first].copy_from_slice(&self.buf[self.head..self.head + first]);
            chunk[first..take].copy_from_slice(&self.buf[..take - first]);
            // SAFETY: single-threaded; the send is an endpoint buffer copy.
            if unsafe { fnb58_usb_send(chunk.as_ptr(), take as u16) } != 0 {
                return false; // endpoint busy: retry on the next tick
            }
            self.head = (self.head + take) % self.buf.len();
            self.len -= take;
        }
        false
    }
}

/// r139: pd_task -> console lines. A tiny CS-mutex ring buffer written by
/// the PD task (`pd_log`) and drained by the main loop into the Sender
/// queue (behind the boot-ring flush, like everything else); the oldest
/// line is dropped when full. r156: 8 -> 24 slots — a session's burst
/// (diag + start + engine + 6 caps lines) overflowed 8 and silently
/// dropped the engine-end line, the one that carries the diagnosis.
struct PdLog {
    lines: [[u8; 96]; 24],
    lens: [u8; 24],
    head: usize,
    count: usize,
}
static PD_LOG: embassy_sync::blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    core::cell::RefCell<PdLog>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(PdLog {
    lines: [[0; 96]; 24],
    lens: [0; 24],
    head: 0,
    count: 0,
}));

/// Log one PD lifecycle line (called from pd_task). r157: the 64-byte
/// limit silently dropped the engine-end line whenever it carried the
/// TX-timeout evidence fields (~78 bytes) — the exact sessions that
/// needed diagnosing. 96 covers the worst case.
pub(crate) fn pd_log(line: &[u8]) {
    if line.is_empty() || line.len() > 96 {
        return;
    }
    PD_LOG.lock(|cell| {
        let mut log = cell.borrow_mut();
        if log.count == log.lines.len() {
            log.head = (log.head + 1) % log.lines.len();
            log.count -= 1;
        }
        let slot = (log.head + log.count) % log.lines.len();
        log.lines[slot] = [0; 96];
        log.lines[slot][..line.len()].copy_from_slice(line);
        log.lens[slot] = line.len() as u8;
        log.count += 1;
    });
}

/// r139 stall watch: the main loop stamps MAIN_STAGE/MAIN_SEQ every pass;
/// SysTick probes periodically and, if the stamp did not move across two
/// probes, dumps both tasks' stages once over USB (best-effort raw send —
/// the loop that normally owns the console is the thing that is stuck).
/// MAIN_STAGE: 1 = doorbell wait, 2 = loop body, 3 = accel BUS lock,
/// 4 = meter BUS lock. pd_task::STAGE: see pd_task.rs.
static MAIN_STAGE: AtomicU8 = AtomicU8::new(0);
static MAIN_SEQ: AtomicU32 = AtomicU32::new(0);
static WATCH_FIRED: AtomicBool = AtomicBool::new(false);
static WATCH_LAST: AtomicU32 = AtomicU32::new(u32::MAX);
static SYSTICKS: AtomicU32 = AtomicU32::new(0);

/// CDC line queue. The vendor send rejects a second call while one is in
/// flight, so every console line the main loop produces (meter status,
/// button events, PD capabilities) is queued here and drained one line
/// per tick — one drain point guarantees the sends never collide.
/// r140: slots are 128 bytes. The old 64-byte limit silently dropped the
/// ~95-byte diag heartbeat (line.len() > 64) — since r119-ish NO heartbeat
/// ever reached the host, which masqueraded as "PC: no log" and a phantom
/// console stall. usbd_ept_send splits >64-byte lines into maxpacket
/// chunks itself (usbd_int.c continues the remainder on IN-complete).
pub(crate) struct Sender {
    queue: [[u8; 128]; 20],
    qhead: usize,
    qlen: usize,
    /// Send attempts rejected by the busy endpoint since last take_fails().
    fails: u32,
}

impl Sender {
    fn new() -> Self {
        Self {
            queue: [[0; 128]; 20],
            qhead: 0,
            qlen: 0,
            fails: 0,
        }
    }

    pub(crate) fn push(&mut self, line: &[u8]) {
        if self.qlen == self.queue.len() || line.len() > 128 {
            return; // backlog or oversize: drop, the next report suffices
        }
        let slot = (self.qhead + self.qlen) % self.queue.len();
        self.queue[slot] = [0; 128];
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
pub(crate) fn push_dec(buf: &mut [u8], pos: usize, v: u32) -> usize {
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

/// FNB58 command checksum: CRC-8, poly 0x39, init 0x42, no reflection,
/// no xorout — over bytes 1..62 of a 64-byte HID report, compared to
/// byte 63 (the OpenFNB58 tools build their commands the same way).
fn crc8_fnb58(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x42;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x39
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Append `v` as two lowercase hex digits.
pub(crate) fn push_hex8(buf: &mut [u8], pos: usize, v: u32) -> usize {
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
    // r139 stall watch: probe MAIN_SEQ periodically; if it did not move
    // across two probes (and the loop had started), name the blocking
    // stage of BOTH tasks once, straight from the exception. The raw send
    // is best-effort: the loop that owns the console is the stuck one.
    let n = SYSTICKS.fetch_add(1, Ordering::Relaxed);
    if n % 2048 == 0 {
        let seq = MAIN_SEQ.load(Ordering::Relaxed);
        let last = WATCH_LAST.swap(seq, Ordering::Relaxed);
        if seq != 0 && seq == last && !WATCH_FIRED.swap(true, Ordering::Relaxed) {
            let mut d = [0u8; 40];
            let mut w = 0;
            for &b in b"[watch] main=" {
                d[w] = b;
                w += 1;
            }
            w = push_dec(&mut d, w, u32::from(MAIN_STAGE.load(Ordering::Relaxed)));
            for &b in b" pd=" {
                d[w] = b;
                w += 1;
            }
            w = push_dec(&mut d, w, u32::from(pd_task::STAGE.load(Ordering::Relaxed)));
            d[w] = b'\r';
            d[w + 1] = b'\n';
            w += 2;
            unsafe {
                fnb58_usb_send(d.as_ptr(), w as u16);
            }
        }
    }
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
fn panic(info: &core::panic::PanicInfo) -> ! {
    // r139: best-effort panic evidence before the halt — the location as
    // one raw console line (the boot ring dies with the board, USB may
    // still be up; a failed send costs nothing, we halt anyway).
    if let Some(location) = info.location() {
        let mut d = [0u8; 96];
        let mut w = 0;
        for &b in b"[panic] " {
            d[w] = b;
            w += 1;
        }
        // Keep the tail of long workspace paths.
        let file = location.file().as_bytes();
        let file = &file[file.len().saturating_sub(70)..];
        for &b in file {
            d[w] = b;
            w += 1;
        }
        d[w] = b':';
        w += 1;
        let mut digits = [0u8; 6];
        let mut t = 0;
        let mut line_no = location.line();
        while line_no > 0 && t < digits.len() {
            digits[t] = b'0' + (line_no % 10) as u8;
            line_no /= 10;
            t += 1;
        }
        while t > 0 {
            t -= 1;
            d[w] = digits[t];
            w += 1;
        }
        d[w] = b'\r';
        d[w + 1] = b'\n';
        w += 2;
        unsafe {
            fnb58_usb_send(d.as_ptr(), w as u16);
        }
    }
    fault_halt()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    fault_halt()
}
