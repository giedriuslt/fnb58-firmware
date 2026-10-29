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
    // Crystal-first clocks; the round-10 capture proved the HICK-fallback
    // halt left the device invisible (no host event at all), which made
    // every later "Code 43" sighting untrustworthy as a clock-path signal:
    // Device Manager keeps failed entries listed, so a stale one reads as
    // a fresh attach. Round 11 attaches UNCONDITIONALLY — whatever the
    // clock path, the host must see a connect event (USBPcap URBs on the
    // hub), separating fresh attach from stale entries. A crash after
    // this point drops the pull-up (fault handler below), so the visible
    // outcomes are:
    //   - host URBs in the capture, device stays  -> attach + live
    //     protocol exchange on the wire: direct evidence;
    //   - device vanishes                          -> firmware crashed;
    //   - nothing at all                           -> never reached the
    //     connect (pre-init fault).
    let Ok((clocks, crystal)) =
        (unsafe { embassy_at32::clocks::configure_usb(board::HEXT_HZ) })
    else {
        loop {
            cortex_m::asm::nop();
        }
    };
    unsafe { embassy_at32::init(clocks) };
    unsafe { cortex_m::interrupt::enable() };

    // SAFETY: exclusive startup owns the USBFS cell. No NVIC unmasking:
    // the cell is polled from the loop below.
    //
    // The PID encodes the clock path on the wire (visible in a captured
    // Get-Descriptor response even when enumeration fails):
    // 0x5859 = PLL from the crystal (accurate USB clock), 0x5858 = HICK
    // fallback (transmitter out of spec by construction).
    unsafe {
        embassy_at32::usb::init(if crystal { 0x5859 } else { 0x5858 });
    }

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
///
/// Round 13: the fault handler first BLINKS the crash code through the
/// pull-up itself — the device visibly flickers back N times (250 ms on,
/// 250 ms off) and then disappears for good. Codes: 1 = crashed after
/// connect, 2 = crashed servicing a bus reset, 3 = crashed dispatching a
/// SETUP, 4 = crashed queuing a descriptor response. No blink = crashed
/// before the connect. The blink runs on busy-wait delays (no timers in
/// a fault context); at 48-192 MHz a 250 ms tick is a few million nops.
fn fault_halt() -> ! {
    let code = embassy_at32::usb::CRASH_CODE.load(core::sync::atomic::Ordering::Acquire);
    unsafe {
        let ctrl = (0x4000_5c00 + 0x40) as *mut u32;
        let cfg = (0x4000_5c00 + 0x60) as *mut u32;
        // Disconnect first, then blink only CFG.PUO (the pull-up).
        core::ptr::write_volatile(ctrl, core::ptr::read_volatile(ctrl) | 0b10);
        for _ in 0..code {
            // OFF (pull-up disabled): PUO = 1.
            core::ptr::write_volatile(cfg, core::ptr::read_volatile(cfg) | 0b10);
            delay_ms(300);
            // ON (pull-up enabled): PUO = 0.
            core::ptr::write_volatile(cfg, core::ptr::read_volatile(cfg) & !0b10);
            delay_ms(300);
        }
        // OFF forever.
        core::ptr::write_volatile(cfg, core::ptr::read_volatile(cfg) | 0b10);
    }
    loop {
        cortex_m::asm::nop();
    }
}

/// ~300 ms busy wait. 192 MHz, ~5 cycles per loop iteration: 30M loops.
fn delay_ms(_ms: u32) {
    for _ in 0..30_000_000 {
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
