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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Crystal-first clocks: PLL from HEXT for a USB-accurate bit rate,
    // HICK-48 fallback when no crystal oscillates.
    //
    // DIAGNOSTIC CONTRACT (round 8): the HICK fallback never attaches.
    // The HICK's ±1% makes the USB transmitter out of spec, so a fallback
    // build could only ever produce another Code 43 — indistinguishable
    // from a protocol bug. Not attaching makes the clock path visible
    // from outside:
    //   - device appears (even as Code 43)  -> crystal path ran; the USB
    //     bit rate is crystal-accurate and any remaining failure is
    //     protocol-level;
    //   - nothing appears at all            -> HEXT never stabilized on
    //     the AT32 (the board's 8 MHz crystal is elsewhere, e.g. the BLE
    //     module) -> the ACC HICK-trim path is the fix.
    let Ok((clocks, crystal)) =
        (unsafe { embassy_at32::clocks::configure_usb(board::HEXT_HZ) })
    else {
        loop {
            cortex_m::asm::nop();
        }
    };
    if !crystal {
        loop {
            cortex_m::asm::nop();
        }
    }
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

/// Drop the D+ pull-up and spin. A panicking or faulting firmware used to
/// leave the pull-up asserted with the cell NAKing forever — visibly
/// identical to any protocol failure (Code 43). Dropping the pull-up on
/// the way down makes a crash observable: the attached device VANISHES.
fn fault_halt() -> ! {
    unsafe {
        // CTRL.DISUSB (bit 1) + CFG.PUO (bit 1): disconnect + pull-up off.
        let ctrl = (0x4000_5c00 + 0x40) as *mut u32;
        core::ptr::write_volatile(ctrl, core::ptr::read_volatile(ctrl) | 0b10);
        let cfg = (0x4000_5c00 + 0x60) as *mut u32;
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
