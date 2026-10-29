//! FNB58 open-firmware smoke test: USB CDC virtual COM port, purely
//! polled.
//!
//! The clock profile is the AT32 system boot ROM's own crystalless
//! recipe: the whole system (and the USB cell, via MISC3.HICK_TO_USB)
//! runs from the internal 48 MHz HICK. No PLL, no USB divider, no NVIC
//! interrupt configuration — the USB cell is serviced by `usb::poll()`
//! from the main loop, so no interrupt routing can silently break
//! enumeration.
//!
//! The device enumerates as a serial port (VID 0x2e3c PID 0x5859) and
//! echoes received bytes plus a one-per-second `FNB58 alive <n>`
//! heartbeat. If the port appears and echoes, the MCU, flash execution,
//! the 48 MHz HICK clock tree, the USBFS cell and the polling stack all
//! work.

#![no_std]
#![no_main]

mod board;
#[allow(dead_code)]
mod panel;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Boot-ROM-style crystalless clocks: 48 MHz HICK everywhere, USB fed
    // from the same HICK. Everything else measures time against this.
    let clocks = match unsafe { embassy_at32::clocks::configure_hick48() } {
        Ok(clocks) => clocks,
        Err(_) => loop {
            cortex_m::asm::nop();
        },
    };
    unsafe { embassy_at32::init(clocks) };
    unsafe { cortex_m::interrupt::enable() };

    // SAFETY: exclusive startup owns the USBFS cell. No NVIC unmasking:
    // the cell is polled from the loop below.
    unsafe { embassy_at32::usb::init() };

    let mut rx = [0u8; 64];
    let mut last_heartbeat = Instant::now();
    loop {
        // Service the USB cell (SETUP, completions, reset).
        embassy_at32::usb::poll();

        // Echo anything received.
        if let Ok(n) = embassy_at32::usb::try_read(&mut rx) {
            if n > 0 {
                let _ = embassy_at32::usb::write(&rx[..n]).await;
                let _ = embassy_at32::usb::write(b"\r\n").await;
            }
        }

        // Heartbeat: one line per second, whatever the host does.
        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            last_heartbeat = Instant::now();
            static TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
            let tick = TICK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            let mut banner = [0u8; 32];
            let text = format_banner(tick, &mut banner);
            let _ = embassy_at32::usb::write(text).await;
        }

        Timer::after_millis(2).await;
    }
}

/// `FNB58 alive <tick>\r\n` into a fixed buffer.
fn format_banner(tick: u32, buf: &mut [u8; 32]) -> &[u8] {
    const PREFIX: &[u8] = b"FNB58 alive ";
    const CRLF: &[u8] = b"\r\n";
    let mut n = 0;
    for &b in PREFIX {
        buf[n] = b;
        n += 1;
    }
    // Decimal tick, no alloc, no formatting machinery.
    if tick == 0 {
        buf[n] = b'0';
        n += 1;
    } else {
        let mut digits = [0u8; 10];
        let mut count = 0;
        let mut value = tick;
        while value > 0 {
            digits[count] = b'0' + (value % 10) as u8;
            value /= 10;
            count += 1;
        }
        while count > 0 {
            count -= 1;
            buf[n] = digits[count];
            n += 1;
        }
    }
    for &b in CRLF {
        buf[n] = b;
        n += 1;
    }
    &buf[..n]
}

#[cortex_m_rt::exception]
fn SysTick() {
    embassy_at32::time_driver::systick_tick();
}
