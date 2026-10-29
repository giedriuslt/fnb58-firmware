//! FNB58 open-firmware smoke test: USB CDC virtual COM port.
//!
//! The device enumerates as a serial port (VID 0x2e3c PID 0x5859) and
//! echoes everything it receives, appending a one-per-second heartbeat
//! line with a tick counter. If the port appears and echoes, the MCU,
//! flash execution, the 240 MHz clock tree, the 48 MHz USB clock, the USB
//! cell and its interrupt all work — the complete bring-up baseline,
//! independent of the (still unverified) display pins.
//!
//! The display transport (`panel`) is kept for the next stage but not
//! started: its pin map is unverified and its failure must not block the
//! USB proof.

#![no_std]
#![no_main]

mod board;
#[allow(dead_code)]
mod panel;

use embassy_at32::pac::at32f403a::interrupt;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Clocks first: everything else measures time against HCLK.
    let clocks = match unsafe { embassy_at32::clocks::configure(board::CLOCK) } {
        Ok(clocks) => clocks,
        Err(_) => loop {
            cortex_m::asm::nop();
        },
    };
    unsafe { embassy_at32::init(clocks) };

    // 48 MHz for the USB cell from the 240 MHz PLL (240 / 5).
    if unsafe { embassy_at32::usb::enable_usb_clock(clocks.sclk_hz) }.is_err() {
        loop {
            cortex_m::asm::nop();
        }
    }

    // The USBFS shares NVIC lines with CAN1; the H line is our interrupt.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_H_CAN1_TX);
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_L_CAN1_RX0);
    }
    unsafe { cortex_m::interrupt::enable() };

    // SAFETY: exclusive startup owns the USBFS cell.
    unsafe { embassy_at32::usb::init() };

    let mut tick: u32 = 0;
    let mut rx = [0u8; 64];
    loop {
        match embassy_at32::usb::read(&mut rx).await {
            Ok(0) => {}
            Ok(n) => {
                let _ = embassy_at32::usb::write(&rx[..n]).await;
                let _ = embassy_at32::usb::write(b"\r\n").await;
            }
            // Not configured or timed out: keep the loop alive either way.
            Err(_) => {}
        }
        // Heartbeat: one line per second, whatever the host does.
        let mut banner = [0u8; 32];
        let text = format_banner(tick, &mut banner);
        let _ = embassy_at32::usb::write(text).await;
        tick = tick.wrapping_add(1);
        Timer::after_millis(1000).await;
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

#[cortex_m_rt::interrupt]
fn USBFS_H_CAN1_TX() {
    embassy_at32::usb::on_interrupt();
}

#[cortex_m_rt::interrupt]
fn USBFS_L_CAN1_RX0() {
    embassy_at32::usb::on_interrupt();
}
