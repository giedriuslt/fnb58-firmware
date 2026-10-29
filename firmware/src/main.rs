//! FNB58 open-firmware smoke test: USB CDC virtual COM port running the
//! **vendor's own C USB stack** (`usbd_core`/`usbd_int`/`at32f403a_407_usb`
//! + the vendor CDC class, compiled by `build.rs` into the firmware).
//!
//! After ten rounds of register-level debugging of a hand-rolled Rust USB
//! stack (each fix real, the symptom never changing), the stack itself was
//! retired: this firmware runs the vendor middleware verbatim, the same
//! code that drives Artery's own examples on this exact silicon.
//!
//! Clock: 192 MHz PLL from the board's 8 MHz crystal with USB /4 — the
//! exact configuration of the vendor `virtual_comport` example, which was
//! confirmed enumerating on this board (round 15 control experiment). The
//! vendor C stack handles the cell; this main only configures clocks,
//! unmasks the IRQs and echoes via the vendor `usb_vcp` API.

#![no_std]
#![no_main]

mod board;
#[allow(dead_code)]
mod panel;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};

// --- Vendor C stack glue (compiled by build.rs into libvendorusb.a) ---

unsafe extern "C" {
    fn fnb58_usb_init();
    fn fnb58_usb_recv(buffer: *mut u8) -> u16;
    fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
}

/// Called from the vendor IRQ handlers; a marker for future diagnostics.
#[unsafe(no_mangle)]
extern "C" fn fnb58_usbd_event_hook() {}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Clock: 192 MHz PLL from the 8 MHz crystal with USB /4 — the exact
    // configuration of the vendor virtual_comport example, which was
    // confirmed working on this board (COM6). Round 14's HICK+ACC path is
    // retired; the crystal is proven present.
    let Ok((clocks, _crystal)) =
        (unsafe { embassy_at32::clocks::configure_usb(board::HEXT_HZ) })
    else {
        loop {
            cortex_m::asm::nop();
        }
    };
    unsafe { embassy_at32::init(clocks) };
    unsafe { cortex_m::interrupt::enable() };

    // The vendor stack is interrupt-driven: unmask both USBFS lines (the
    // vendor example uses the L line; H is wired identically) and the ACC
    // calibration line. The C handlers override the PAC's weak vectors.
    use embassy_at32::pac::at32f403a::interrupt;
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_L_CAN1_RX0);
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_H_CAN1_TX);
    }

    // Vendor init: CRM USB clock enable, cell reset, usbd_core_init with
    // the CDC class + descriptors, connect.
    unsafe {
        fnb58_usb_init();
    }

    let mut rx = [0u8; 64];
    let mut last_heartbeat = Instant::now();
    loop {
        // Echo anything received (the vendor class buffers OUT data).
        let n = unsafe { fnb58_usb_recv(rx.as_mut_ptr()) } as usize;
        if n > 0 {
            unsafe {
                fnb58_usb_send(rx.as_ptr(), n as u16);
                fnb58_usb_send(b"\r\n".as_ptr(), 2);
            }
        }

        // Heartbeat: one line per second, whatever the host does.
        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            last_heartbeat = Instant::now();
            static TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
            let tick = TICK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            let mut banner = [0u8; 32];
            let text = format_banner(tick, &mut banner);
            unsafe {
                fnb58_usb_send(text.as_ptr(), text.len() as u16);
            }
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
